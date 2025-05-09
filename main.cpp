#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtx/norm.hpp>



#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cmath>

struct Vertex {
    float x, y, z;
    float u, v;
};

struct Triangle {
    unsigned int v1, v2, v3;
    unsigned int n1, n2, n3; 
};

struct Normal {
    float x, y, z;
};

struct Vec2{
    float x, y; 
};

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}


// Flattens 3D triangle (by indices into the Vertex array) to 2D using orthonormal basis
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
    if (uvs.empty()) return;

    Vec2 minUV = uvs[0];
    Vec2 maxUV = uvs[0];

    // Find bounding box
    for (const auto& uv : uvs) {
        minUV.x = std::min(minUV.x, uv.x);
        minUV.y = std::min(minUV.y, uv.y);
        maxUV.x = std::max(maxUV.x, uv.x);
        maxUV.y = std::max(maxUV.y, uv.y);
    }

    Vec2 scale = { maxUV.x - minUV.x, maxUV.y - minUV.y };

    // Avoid divide-by-zero in degenerate cases
    if (scale.x == 0.0f) scale.x = 1.0f;
    if (scale.y == 0.0f) scale.y = 1.0f;

    // Normalize to [0, 1]
    for (auto& uv : uvs) {
        uv.x = (uv.x - minUV.x) / scale.x;
        uv.y = (uv.y - minUV.y) / scale.y;
    }
}

void solveLSCM(const std::vector<Vertex>& vertices, const std::vector<Triangle>& triangles, int anchor1, Vec2 uv1, int anchor2, Vec2 uv2, std::vector<Vec2>& uvs) {
    //set up Ax = b A is the sparse matrix b is right hand side
    const int n = vertices.size();
    std::vector<std::vector<double>> A(2 * n, std::vector<double>(2 * n, 0.0));
    std::vector<double> b(2 * n, 0.0);

    for (const Triangle& tri : triangles) { //for each triangle
        int i = tri.v1;
        int j = tri.v2;
        int k = tri.v3;

        Vec2 z1, z2; //triangle sides in 2d complex space
        flattenTriangle(vertices[i], vertices[j], vertices[k], z1, z2); //get the uv coords of the rest of the triangle

        double z_a = z1.x, z_b = z1.y;
        double z_c = z2.x, z_d = z2.y;

        double area = std::abs((z_a * z_d - z_b * z_c) / 2.0);
        if (area < 1e-8) continue; //degenerate triangles

        //matrix representing the triangle
        double S[3][2] = {
            {-(z_a - z_c), -(z_b - z_d)}, //s0
            {z_a, z_b}, //s1
            {z_c, z_d} //s2
        };

        int ids[3] = {i, j, k};
        //apply triangle to A
        for (int r = 0; r < 3; ++r) {
            for (int s = 0; s < 3; ++s) {
                for (int t = 0; t < 2; ++t) {
                    for (int u = 0; u < 2; ++u) {
                        A[2 * ids[r] + t][2 * ids[s] + u] += S[r][t] * S[s][u];
                    }
                }
            }
        }
    }

    // Apply anchor constraints
    for (int k = 0; k < 2; ++k) {
        int idx = (k == 0 ? anchor1 : anchor2);
        Vec2 fixedUV = (k == 0 ? uv1 : uv2);

        for (int j = 0; j < 2 * n; ++j) {
            A[2 * idx + 0][j] = 0;
            A[2 * idx + 1][j] = 0;
        }

        A[2 * idx + 0][2 * idx + 0] = 1;
        A[2 * idx + 1][2 * idx + 1] = 1;

        b[2 * idx + 0] = fixedUV.x;
        b[2 * idx + 1] = fixedUV.y;
    }

    // Solve Ax = b using basic Gauss elimination
    std::vector<double> x(2 * n);
    for (int i = 0; i < 2 * n; ++i) {
        double pivot = A[i][i];
        if (std::abs(pivot) < 1e-8) continue;

        for (int j = i; j < 2 * n; ++j)
            A[i][j] /= pivot;
        b[i] /= pivot;

        for (int k = 0; k < 2 * n; ++k) {
            if (k == i) continue;
            double f = A[k][i];
            for (int j = i; j < 2 * n; ++j)
                A[k][j] -= f * A[i][j];
            b[k] -= f * b[i];
        }
    }

    // Extract UVs
    uvs.resize(n);
    for (int i = 0; i < n; ++i) {
        uvs[i].x = b[2 * i];
        uvs[i].y = b[2 * i + 1];
    }
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

            auto parseFace = [](const std::string& s, unsigned int& vertexIndex, unsigned int& normalIndex) {
                // Split by '/' 1/2/3
                size_t firstSlash = s.find('/');
                size_t secondSlash = s.find('/', firstSlash + 1);

                // Vertex index (before first slash)
                vertexIndex = static_cast<unsigned int>(std::stoi(s.substr(0, firstSlash))) - 1;

                // Normal index (after second slash, if exists)
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

int main(int argc, char** argv) {   
    // Validate args
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <model.obj>\n";
        return 1;
    }

    std::string objFilename = argv[1];

    // std::ifstream file(objFilename);
    // std::string line;
    // while (std::getline(file, line)) {
    //     std::cout << line << std::endl;
    // }

    // Initialize GLFW
    if (!glfwInit()) return -1;
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "OBJ Viewer", NULL, NULL);
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

    int anchor1 = 0, anchor2 = 0;
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();

    for (int i = 0; i < vertices.size(); ++i) {
        if (vertices[i].x < minX) {
            minX = vertices[i].x;
            anchor1 = i;
        }
        if (vertices[i].x > maxX) {
            maxX = vertices[i].x;
            anchor2 = i;
        }
    }

    Vec2 uv1 = {0.0f, 0.0f};
    Vec2 uv2 = {1.0f, 1.0f};
    anchor2 = 7;
    std::vector<Vec2> uvs;
    printf("anchor1:%d and anchor2:%d\n", anchor1, anchor2);
    solveLSCM(vertices, triangles, anchor1, uv1, anchor2, uv2, uvs);

    normalizeUVs(uvs);
    for(auto uv : uvs){
        printf("u:%f v:%f\n", uv.x, uv.y);
    }

    



    // Create buffers
    GLuint VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    // Bind VAO
    glBindVertexArray(VAO);

    // Vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex), vertices.data(), GL_STATIC_DRAW);

    // Index buffer
    std::vector<unsigned int> indices;
    for (auto& tri : triangles) {
        indices.push_back(tri.v1);
        indices.push_back(tri.v2);
        indices.push_back(tri.v3);
    }

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    // Vertex attributes
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);
    glEnableVertexAttribArray(0);

    // Simple render loop
    while (!glfwWindowShouldClose(window)) {
        glClearColor(0.1f, 0.1f, 0.2f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, indices.size(), GL_UNSIGNED_INT, 0);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    // Cleanup
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glDeleteVertexArrays(1, &VAO);
    glfwTerminate();
    return 0;
}
