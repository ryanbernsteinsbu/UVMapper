#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>


#include <iostream>
#include <map>
#include <unordered_map>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>
#include <queue>
#include <eigen3/Eigen/Dense>
#include <set>
#include <iomanip>

#define maxDihedralAngle 116.566 //anything as sharp or less sharp than a dodecahedron is cut
#define anchors 2

struct Vertex { 
    float x, y, z;
    float u, v; //to store info after
};

struct Triangle { //triangles store normal information index
    unsigned int v1, v2, v3;
    unsigned int n1, n2, n3; 
};

struct Normal {
    float x, y, z; //vn info
};

struct Vec2{
    float x, y; //simple vec 2 for operations
};

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

// aUV is in [0,1], remaps to NDC [-1,1] (just slighlty modified this)
const char* uvVertSrc = R"(
    #version 330 core
    layout(location = 0) in vec2 aUV;
    void main() {
        // map [0,1] to [-1,1]
        vec2 pos = aUV * 2.0 - 1.0;
        gl_Position = vec4(pos, 0.0, 1.0);
    }
)";
    
// just draw white lines or flat color (i did not code this)
const char* uvFragSrc = R"(
    #version 330 core
    out vec4 FragColor;
    void main() {
        FragColor = vec4(1.0); 
    }
)";

GLuint compileShader(GLenum type, const char* src) { //complile shaders (did not code this)
    GLuint s = glCreateShader(type);
    glShaderSource(s, 1, &src, nullptr);
    glCompileShader(s);
    GLint ok; glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char buf[512]; glGetShaderInfoLog(s, 512, nullptr, buf);
        std::cerr << "Shader compile error: " << buf << "\n";
    }
    return s;
}

GLuint linkProgram(GLuint v, GLuint f) { // link compiled shaders (did not code this)
    GLuint p = glCreateProgram();
    glAttachShader(p, v);
    glAttachShader(p, f);
    glLinkProgram(p);
    GLint ok; glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char buf[512]; glGetProgramInfoLog(p, 512, nullptr, buf);
        std::cerr << "Program link error: " << buf << "\n";
    }
    return p;
}

//pre-processing
struct EdgeKey { //key for graph in map
    int v0, v1;
    EdgeKey(int a, int b) {
        if (a < b) { //undirected graph so (a,b) is = (b,a)
            v0 = a;
            v1 = b;
        }
        else{
            v0 = b;
            v1 = a;
        }
    }
    bool operator<(EdgeKey const& o) const { //overload comparison for map structure
        return v0 < o.v0 || (v0 == o.v0 && v1 < o.v1);
    }
};

struct DualEdge {
    int f0, f1;
    float weight;
};

void buildEdgeToFacesMap(const std::vector<Triangle>& triangles, std::map<EdgeKey, std::vector<int>>& edgeToFaces){
    edgeToFaces.clear();
    for (int t = 0; t < (int)triangles.size(); t++) {
        const auto& tri = triangles[t];

        EdgeKey e0(tri.v1, tri.v2);
        EdgeKey e1(tri.v2, tri.v3);
        EdgeKey e2(tri.v3, tri.v1);

        edgeToFaces[e0].push_back(t);
        edgeToFaces[e1].push_back(t);
        edgeToFaces[e2].push_back(t);
    }
}

void buildFaceNormals(const std::vector<Triangle>& triangles, const std::vector<Vertex>& vertices ,std::vector<glm::vec3>& faceNormals){
    for(int i = 0; i < (int)triangles.size(); i++){
        Triangle tri = triangles[i];

        glm::vec3 p0 {vertices[tri.v1].x, vertices[tri.v1].y, vertices[tri.v1].z};
        glm::vec3 p1 {vertices[tri.v2].x, vertices[tri.v2].y, vertices[tri.v2].z};
        glm::vec3 p2 {vertices[tri.v3].x, vertices[tri.v3].y, vertices[tri.v3].z};

        glm::vec3 normal = glm::normalize(glm::cross((p1 - p0),(p2 - p0)));
        // std::cout << glm::to_string(normal) << "\n";

        faceNormals.push_back(normal);
    }
}


std::vector<DualEdge> computeDualEdges(const std::map<EdgeKey, std::vector<int>>& edgeToFaces, const std::vector<glm::vec3>& faceNormals){
    std::vector<DualEdge> dualEdges;
    dualEdges.reserve(edgeToFaces.size());

    for (auto const& kv : edgeToFaces) {
        auto const& faces = kv.second;
        
        if (faces.size() != 2) continue;

        int fA = faces[0];
        int fB = faces[1];

        
        float angleCos = glm::clamp(glm::dot(faceNormals[fA], faceNormals[fB]), -1.0f, 1.0f);

        // from 0 to 2
        float w = 1.0f - angleCos;
        // printf("face1: %d, face2: %d, weight %f\n", fA, fB, w);

        dualEdges.push_back({ fA, fB, w });
    }

    return dualEdges;
}

std::vector<DualEdge> primMST(int numFaces, const std::vector<DualEdge>& dualEdges){ 
    // for (auto const& e : dualEdges) {
    //     assert(e.f0 >= 0 && e.f0 < numFaces);
    //     assert(e.f1 >= 0 && e.f1 < numFaces);
    // }
    std::vector<std::vector<std::pair<int,float>>> graph(numFaces);
    for (auto const& e : dualEdges) {
        graph[e.f0].emplace_back(e.f1, e.weight);
        graph[e.f1].emplace_back(e.f0, e.weight);
    }

    std::vector<bool> inMST(numFaces, false);
    std::vector<DualEdge> mst;
    mst.reserve(numFaces - 1);

    using PQItem = std::tuple<float,int,int>; //I found this implementation, did not come up with it
    struct cmp {
        bool operator()(PQItem const& a, PQItem const& b) const {
            return std::get<0>(a) > std::get<0>(b);
        }
    };
    std::priority_queue<PQItem, std::vector<PQItem>, cmp> pq;

    inMST[0] = true;
    for (auto const& adj : graph[0])
        pq.emplace(adj.second, 0, adj.first);

    while (!pq.empty() && mst.size() < (size_t)numFaces - 1) {
        auto [w, u, v] = pq.top();
        pq.pop();
        if (inMST[v]) continue;// already in tree

        inMST[v] = true;
        mst.push_back({u, v, w});

        for (auto const& adj : graph[v]) {
            if (!inMST[adj.first])
                pq.emplace(adj.second, v, adj.first);
        }
    }

    return mst;
}

std::vector<std::vector<int>> extractIslands(int numFaces, const std::vector<DualEdge>& mstEdges, float maxAngleDeg){
    
    float maxAngleRad = maxAngleDeg * (3.14159265f / 180.0f);
    float weightThresh = 1.0f + std::cos(maxAngleRad); // get weigth threshold
    // printf("weight thresh: %f\n", weightThresh);

    std::vector<std::vector<int>> adj(numFaces); //add faces from valid edges
    for (auto const& e : mstEdges) {
        // printf("e1: %d, e2: %d, weight: %f\n", e.f0, e.f1, e.weight);
        if (e.weight <= weightThresh) {
            adj[e.f0].push_back(e.f1);
            adj[e.f1].push_back(e.f0);
        }
    }

    // fill components (algorithm is not mine)
    std::vector<bool> visited(numFaces, false);
    std::vector<std::vector<int>> islands;
    islands.reserve(numFaces);

    for (int f = 0; f < numFaces; f++) {
        if (visited[f]) continue;
        std::vector<int> queue = {f};
        visited[f] = true;

        for (size_t qi = 0; qi < queue.size(); qi++) {
            int cur = queue[qi];
            for (int nbr : adj[cur]) {
                if (!visited[nbr]) {
                    visited[nbr] = true;
                    queue.push_back(nbr);
                }
            }
        }

        islands.push_back(std::move(queue));
    }

    if (islands.size() == 1) { //we need to split at least once did not code this
        std::vector<std::vector<int>> mstAdj(numFaces);
        for (auto const& e : mstEdges) {
            mstAdj[e.f0].push_back(e.f1);
            mstAdj[e.f1].push_back(e.f0);
        }
    
        auto subtreeSize = [&](int start, int blockU, int blockV) {
            std::vector<bool> vis(numFaces,false);
            std::queue<int> q;
            q.push(start);
            vis[start] = true;
            int cnt = 0;
            while (!q.empty()) {
                int u = q.front();
                q.pop();
                cnt++;
                for (int w : mstAdj[u]) {
                    // skip the removed edge
                    if ((u == blockU && w == blockV) || (u == blockV && w == blockU)) continue;
                    if (!vis[w]) {
                        vis[w] = true;
                        q.push(w);
                    }
                }
            }
            return cnt;
        };
    
        int bestU = -1, bestV = -1;
        int target = numFaces / 2;
        int bestDiff = numFaces;
        for (auto const& e : mstEdges) {
            int s = subtreeSize(e.f0, e.f0, e.f1);
            int diff = std::abs(s - (numFaces - s));
            if (diff < bestDiff) {
                bestDiff = diff;
                bestU = e.f0;
                bestV = e.f1;
            }
        }
    
        islands.clear();
        std::vector<bool> visited(numFaces,false);
        for (int seed : {bestU, bestV}) {
            if (visited[seed]) continue;
            std::vector<int> comp;
            std::queue<int> q;
            q.push(seed);
            visited[seed] = true;
            while (!q.empty()) {
                int u = q.front();
                q.pop();
                comp.push_back(u);
                for (int w : mstAdj[u]) {
                    if ((u == bestU && w == bestV) || (u == bestV && w == bestU)) continue;
                    if (!visited[w]) {
                        visited[w] = true;
                        q.push(w);
                    }
                }
            }
            islands.push_back(std::move(comp));
        }
    }


    return islands;
}


void flattenTriangle(const Vertex& p0, const Vertex& p1, const Vertex& p2, Vec2& z1, Vec2& z2) {
    glm::vec3 u = glm::vec3(p1.x, p1.y, p1.z) - glm::vec3(p0.x, p0.y, p0.z);
    glm::vec3 v = glm::vec3(p2.x, p2.y, p2.z) - glm::vec3(p0.x, p0.y, p0.z);

    glm::vec3 e1 = glm::normalize(u); //basis vector in direction of u
    glm::vec3 n = glm::normalize(glm::cross(u, v)); //face normal of triangle
    glm::vec3 e2 = glm::cross(n, e1); // use face normal to get other basis vector

    z1.x = glm::dot(u, e1); 
    z1.y = glm::dot(u, e2);

    z2.x = glm::dot(v, e1);
    z2.y = glm::dot(v, e2);
}

void normalizeUVs(std::vector<Vec2>& uvs) {
    if (uvs.empty()) return; //empty case

    Vec2 minUV = uvs[0];
    Vec2 maxUV = uvs[0];

    // find bounding box
    for (auto& uv : uvs) {
        minUV.x = std::min(minUV.x, uv.x);
        minUV.y = std::min(minUV.y, uv.y);
        maxUV.x = std::max(maxUV.x, uv.x);
        maxUV.y = std::max(maxUV.y, uv.y);
    }

    Vec2 scale = { maxUV.x - minUV.x, maxUV.y - minUV.y };

    // div by 0
    if (scale.x == 0.0f) scale.x = 1.0f;
    if (scale.y == 0.0f) scale.y = 1.0f;

    // normalize to [0, 1]
    for (auto& uv : uvs) {
        uv.x = (uv.x - minUV.x) / scale.x;
        uv.y = (uv.y - minUV.y) / scale.y;
    }
}

void normalizeUVsUniform(std::vector<Vec2>& uvs) {
    if (uvs.empty()) return; //empty case

    Vec2 minUV = uvs[0];
    Vec2 maxUV = uvs[0];

    for (auto& uv : uvs) {
        minUV.x = std::min(minUV.x, uv.x);
        minUV.y = std::min(minUV.y, uv.y);
        maxUV.x = std::max(maxUV.x, uv.x);
        maxUV.y = std::max(maxUV.y, uv.y);
    }

    float width  = maxUV.x - minUV.x;
    float height = maxUV.y - minUV.y;
    float scale  = std::max(width, height);
    if (scale < 1e-8f) scale = 1.0f;

    // normalize to [0, 1]
    for (auto& uv : uvs) {
        uv.x = (uv.x - minUV.x) / scale;
        uv.y = (uv.y - minUV.y) / scale;
    }
}

std::vector<int> findBoundaryVerts(const std::vector<Triangle>& localTris)
{
    std::map<std::pair<int,int>,int> edgeCount;
    for (auto& T : localTris) {
      unsigned int v[3] = {T.v1,T.v2,T.v3};
      for (int i = 0; i < 3; i++) {
        int a = v[i], b = v[(i + 1) % 3];
        if (a > b) std::swap(a,b);
        edgeCount[{a,b}] += 1;
      }
    }
    
    std::set<int> boundary;
    for (auto& kv : edgeCount) {
      if (kv.second == 1) {
        boundary.insert(kv.first.first);
        boundary.insert(kv.first.second);
      }
    }
    return { boundary.begin(), boundary.end() };
}

void solveLSCM(const std::vector<Vertex>& vertices, const std::vector<Triangle>& triangles, int anchor1, Vec2 uv1, int anchor2, Vec2 uv2, int anchor3, Vec2 uv3, std::vector<Vec2>& uvs) {
    //set up Ax = b A is the sparse matrix b is right hand side
    const int n = vertices.size();
    std::vector<std::vector<double>> A(2 * n, std::vector<double>(2 * n, 0.0));
    std::vector<double> b(2 * n, 0.0);

    for (const Triangle& tri : triangles) { //for each triangle
        int i = tri.v1;
        int j = tri.v2;
        int k = tri.v3;

        Vec2 z1, z2; //triangle sides in 2d complex space
        flattenTriangle(vertices[i], vertices[j], vertices[k], z1, z2); 

        double a1 = z1.x, b1 = z1.y;
        double a2 = z2.x, b2 = z2.y;

        double area = std::abs((a1 * b2 - b1 * a2) / 2.0);
        if (area < 1e-8) continue; //degenerate triangles

        // get positive CR matrix
        int ids[3] = { i, j, k };
        double coeffs[3][2] = {
            { -(a1 + a2),  (b1 + b2) },
            {a1, -b1},
            {a2, -b2}
        };

        for (int r = 0; r < 3; r++) {
            int vr = ids[r];
            for (int s = 0; s < 3; s++) { //put in matrix
                int vs = ids[s];
                A[2 * vr + 0][2 * vs + 0] += coeffs[r][0] * coeffs[s][0]; // uu
                A[2 * vr + 0][2 * vs + 1] += coeffs[r][0] * coeffs[s][1]; // uv
            }
        }
        //imaginary/rotated side
        double icoeffs[3][2] = {
            {-(b1 + b2), -(a1 + a2)}, 
            {b1,a1}, 
            {b2,a2} 
        };

        for (int r = 0; r < 3; ++r) { //put in matrix
            int vr = ids[r];
            for (int s = 0; s < 3; ++s) {
                int vs = ids[s];
                A[2 * vr + 1][2 * vs + 0] += icoeffs[r][0] * icoeffs[s][0]; // vu
                A[2 * vr + 1][2 * vs + 1] += icoeffs[r][0] * icoeffs[s][1]; // vv
            }
        }
    }

    // Apply anchor constraints
    for (int k = 0; k < anchors; k++) {
        int idx = (k == 0 ? anchor1 : (k == 1 ? anchor2 : anchor3));
        Vec2 fixedUV = (k == 0 ? uv1 : (k == 1 ? uv2 : uv3));

        int r0 = 2 * idx + 0;
        int r1 = 2 * idx + 1;

        for (int k = 0; k < 2 * n ;k++) {
            A[r0][k] = 0;    // zero row
            A[r1][k] = 0;
        }
        // now pin them
        A[r0][r0] = 1;
        A[r1][r1] = 1;
        b[r0] = fixedUV.x;
        b[r1] = fixedUV.y;
    }

    Eigen::MatrixXd M(2 * n, 2 * n);
    Eigen::VectorXd B(2 * n);
    for(int i = 0; i < 2 * n; i++){
        B(i) = b[i];
        for(int j = 0; j < 2 * n;j++){
            M(i,j) = A[i][j];
        }
    }

    Eigen::VectorXd X = M.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);

    for(int i = 0; i < 2 * n ;i++){
        b[i] = X(i);
    }

    // get the uvs
    uvs.resize(n);
    for (int i = 0; i < n; i++) {
        uvs[i].x = b[2 * i];
        uvs[i].y = b[2 * i + 1];
    }

    // for (int i = 0; i < uvs.size(); ++i)
        // printf("pre-norm uv[%d] = (%f,%f)\n", i, uvs[i].x, uvs[i].y);
}

std::vector<float> islandSolve(const std::vector<Vertex>& vertices, const std::vector<Triangle>& triangles, const std::vector<std::vector<int>>& islands) { //wrote code, but very similar to online examples
    std::vector<float> uvData;
    uvData.reserve(triangles.size() * 3 * 2);
    // int ran =0;
    for (auto const& island : islands) {
        // if(ran == 0){
        //     ran = 1;
        // } else {
        //     break;
        // }
        if (island.size() <= 2) { //edge cases are underconstrained but work fine hardcode for better performance on large mesh
            

            
            std::vector<int> verts;
            verts.reserve(island.size()*3);
            for (int face : island) {
                auto const& tri = triangles[face];
                for (auto vi : { tri.v1, tri.v2, tri.v3 }) {
                    if (std::find(verts.begin(), verts.end(), vi) == verts.end()) //add each vert once coukd have used set
                        verts.push_back(vi);
                }
            }
            std::unordered_map<int,Vec2> uvMap;
            if (verts.size() == 3) { //tri
                uvMap[verts[0]] = {0,0};
                uvMap[verts[1]] = {1,0};
                uvMap[verts[2]] = {0,1};
            } else if (verts.size() == 4) { //quad
                uvMap[verts[0]] = {0,0};
                uvMap[verts[1]] = {1,0};
                uvMap[verts[2]] = {1,1};
                uvMap[verts[3]] = {0,1};
            }

            for (int face : island) {
                auto const& tri = triangles[face];
                for (auto vi : { tri.v1, tri.v2, tri.v3 }) {
                    Vec2 uvc = uvMap[vi];
                    uvData.push_back(uvc.x);
                    uvData.push_back(uvc.y);
                }
            }
            continue;
        }
        std::unordered_map<int,int> vertMap; //first we gotta map the island verts to global ones 
        vertMap.reserve(island.size() * 3);

        std::vector<Vertex> localVerts;
        std::vector<Triangle> localTris;
        localVerts.reserve(island.size() * 3);
        localTris.reserve(island.size());

        // get all verts
        for (int face : island) {
            auto const& T = triangles[face];
            for (auto origV : { T.v1, T.v2, T.v3 }) {
                if (!vertMap.count(origV)) { //if vertex is new
                    int li = (int)localVerts.size(); //get local index
                    vertMap[origV] = li; 
                    localVerts.push_back(vertices[origV]); //get global coords from index
                }
            }
        }

        for (int face : island) {
            auto const& T = triangles[face];
            Triangle lt;
            lt.v1 = vertMap[T.v1];
            lt.v2 = vertMap[T.v2];
            lt.v3 = vertMap[T.v3];
            localTris.push_back(lt); //add tri from island
        }

        // now that the triangle and vertex arrays are made get the anchors

        int a0 = 0, a1 = 0;
        std::vector<int> boundaryVerts = findBoundaryVerts(localTris);
        //get far pair
        float bestd = -1;
        for (int i = 0; i < boundaryVerts.size(); i++) {
            for (int j = i + 1; j < boundaryVerts.size(); j++) {
            auto vi = localVerts[boundaryVerts[i]];
            auto vj = localVerts[boundaryVerts[j]];
            float d2 = glm::distance2(glm::vec3(vi.x, vi.y, vi.z), glm::vec3(vj.x, vj.y, vj.z));
            if (d2 > bestd) {
                bestd = d2;
                a0 = i;
                a1 = j;
            }
        }
}
        // to avoid degen triangles ill get a third point to kill shear and rotational issues NOT USED
        int a2 = a0;  
        float bestArea = 0;
        glm::vec3 p0 = glm::vec3(localVerts[a0].x, localVerts[a0].y, localVerts[a0].z);
        glm::vec3 p1 = glm::vec3(localVerts[a1].x, localVerts[a1].y, localVerts[a1].z);
        glm::vec3 u  = p1 - p0;

        for (int i = 0; i < (int)boundaryVerts.size(); i++) { //NOT USED
            int bi = boundaryVerts[i];
            if (i == a0 || i == a1) continue;
            glm::vec3 w = glm::vec3(localVerts[bi].x, localVerts[bi].y, localVerts[bi].z) - p0;
            float area = glm::length(glm::cross(u, w));
            if (area > bestArea) {
                bestArea = area;
                a2 = bi;
            }
        }
        // printf("%d %d %d\n",a0,a1,a2);

        Vec2 uvA{0,0}, uvB{1,0}, uvC{0,1}; //ONLY USES a and b, use define to change it (don't its really broken)
        if(anchors == 2){ //fix anchor
            uvB = {1,1};
        }
        std::vector<Vec2> uvout;
        solveLSCM(localVerts, localTris, a0, uvA, a1, uvB, a2, uvC, uvout);

        normalizeUVsUniform(uvout);

        // push each face UVs directly to uvData
        for (int face : island) {
            auto const& T = triangles[face];
            int l0 = vertMap[T.v1];
            int l1 = vertMap[T.v2];
            int l2 = vertMap[T.v3];

            uvData.push_back(uvout[l0].x);
            uvData.push_back(uvout[l0].y);
            uvData.push_back(uvout[l1].x);
            uvData.push_back(uvout[l1].y);
            uvData.push_back(uvout[l2].x);
            uvData.push_back(uvout[l2].y);
        }
    }
    
    return uvData;
}

bool loadOBJ(const std::string& filename, std::vector<Vertex>& vertices, std::vector<Triangle>& triangles, std::vector<Normal>& normals) {
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        // std::cout << line << std::endl;
        std::istringstream ss(line);
        std::string prefix;
        ss >> prefix;
        if (prefix == "v") {
            Vertex v;
            ss >> v.x >> v.y >> v.z;
            vertices.push_back(v);

        } else if (prefix == "vn") {
            Normal n;
            ss >> n.x >> n.y >> n.z;
            normals.push_back(n);

        } else if (prefix == "f") {
            Triangle t;
            std::string v1, v2, v3;
            ss >> v1 >> v2 >> v3;

            auto parseFace = [](const std::string& s, unsigned int& vertexIndex, unsigned int& normalIndex) { //did not make this

                size_t firstSlash = s.find('/');
                size_t secondSlash = s.find('/', firstSlash + 1);


                vertexIndex = static_cast<unsigned int>(std::stoi(s.substr(0, firstSlash))) - 1;


                if (secondSlash != std::string::npos) {
                    normalIndex = static_cast<unsigned int>(std::stoi(s.substr(secondSlash + 1))) - 1;
                }
            };

            // Parse the face vertices and normals
            parseFace(v1, t.v1, t.n1);
            parseFace(v2, t.v2, t.n2);
            parseFace(v3, t.v3, t.n3);

            triangles.push_back(t);
        }
    }
    return true;
}

bool writeOBJwithUV(const std::string &outName, std::vector<Vertex>& vertices, std::vector<Triangle>& triangles, std::vector<Normal>& normals, std::vector<float>& uvs) {
    std::ofstream out(outName);
    if (!out.is_open()) {
        std::cerr << "Failed to open " << outName << " for writing\n";
        return false;
    }

    for (auto const& v : vertices) {
        out << "v " << v.x << " " << v.y << " " << v.z << "\n";
    }

    for (int i = 0; i < (int)uvs.size(); i+=2) {
        out << "vt " << uvs[i] << " " << uvs[i + 1] << "\n";
    }

    bool haveNormals = !normals.empty();
    if (haveNormals) {
      for (auto &n : normals)
        out << "vn " << n.x<<" "<<n.y<<" "<<n.z<<"\n";
    }

    int cornerIndex = 1;  
    for (auto const& T : triangles) {
        // vertices
        unsigned v1 = T.v1 + 1;
        unsigned v2 = T.v2 + 1;
        unsigned v3 = T.v3 + 1;


        out << "f ";
        // corner 0
        out << v1 << "/" << cornerIndex;
        // if (haveNormals) out << "/" << n1; removed vertnormal index
        out << " ";
        // corner 1
        out << v2 << "/" << (cornerIndex+1);
        // if (haveNormals) out << "/" << n2;
        out << " ";
        // corner 2
        out << v3 << "/" << (cornerIndex+2);
        // if (haveNormals) out << "/" << n3;
        out << "\n";

        cornerIndex += 3;
    }

    out.close();
    return true;
}

int main(int argc, char** argv) {   
    // Validate args
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <model.obj>\n";
        return 1;
    }

    std::string objFilename = argv[1];

    // Initialize GLFW (none of the GLFW or OPENGL code is done by me)
    if (!glfwInit()) return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(800, 800, "OBJ Viewer", NULL, NULL);
    if (!window) {
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Load OpenGL functions using GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) return -1;



    // Load model
    std::vector<Vertex> vertices;
    std::vector<Triangle> triangles;
    std::vector<Normal> normals;
    if (!loadOBJ(objFilename, vertices, triangles, normals)) {
        std::cerr << "Failed to load OBJ\n";
        return -1;
    }
    //begin solving for lscm

    std::map<EdgeKey,std::vector<int>> edgeToFaces; //create edge: (face&face) map
    std::vector<glm::vec3> faceNormals; //get all face normals

    buildEdgeToFacesMap(triangles, edgeToFaces); //populate data structures
    buildFaceNormals(triangles, vertices, faceNormals);

    std::vector<DualEdge> dualEdges = computeDualEdges(edgeToFaces, faceNormals); //generate (face, face, weight) tuples for graph
    
    auto mstEdges  = primMST((int) triangles.size(), dualEdges); //get mst
    // for(DualEdge d: mstEdges){
    //     printf("%d, %d\n",d.f0, d.f1);
    // }
    auto islands = extractIslands((int)triangles.size(), mstEdges, maxDihedralAngle); //break up mst

    // for(const auto& island: islands){
    //     for(int faceID: island){
    //         std::cout << faceID << "\n";
    //     }
    //     std::cout << "\n";
    // }
    
    auto uvData = islandSolve(vertices, triangles, islands);
    
    // for(int i = 0; i < uvData.size(); i += 2){
    //     printf("u:%f v:%f\n", uvData[i], uvData[i + 1]);
    //     // printf("%d\n", i);
    //     if((i + 2) % 6 == 0){
    //         printf("\n");
    //     }
    // }
    if (!writeOBJwithUV("resources/out.obj", vertices, triangles, normals, uvData)) {
        std::cerr << "Failed to generate OBJ\n";
        return -1;
    }

    GLuint uvVert = compileShader(GL_VERTEX_SHADER,   uvVertSrc);
    GLuint uvFrag = compileShader(GL_FRAGMENT_SHADER, uvFragSrc);
    GLuint uvProgram = linkProgram(uvVert, uvFrag);

    //Build UV VAO
    GLuint uvVAO, uvVBO;
    glGenVertexArrays(1, &uvVAO);
    glGenBuffers(1, &uvVBO);

    glBindVertexArray(uvVAO);
        glBindBuffer(GL_ARRAY_BUFFER, uvVBO);
        glBufferData(GL_ARRAY_BUFFER,
                uvData.size() * sizeof(float),
                uvData.data(),
                GL_STATIC_DRAW);

        // attribute 0 = vec2 aUV
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    GLsizei uvVertexCount = static_cast<GLsizei>(uvData.size() / 2);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    // Simple render loop
    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
    
        // Draw UV map
        glUseProgram(uvProgram);
        glBindVertexArray(uvVAO);
        glDrawArrays(GL_TRIANGLES, 0, uvVertexCount);
    
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    glDeleteBuffers(1, &uvVBO);
    glDeleteVertexArrays(1, &uvVAO);

    glDeleteProgram(uvProgram);

    glfwTerminate();
    return 0;
}
