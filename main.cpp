#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

struct Vertex {
    float x, y, z;
};

struct Triangle {
    unsigned int v1, v2, v3;
    unsigned int n1, n2, n3; 
};

struct Normal {
    float x, y, z;
};

void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

bool loadOBJ(const std::string& filename, std::vector<Vertex>& vertices, std::vector<Triangle>& triangles, std::vector<Normal>& normals) {
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line)) {
        std::cout << line << std::endl;
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
    if (!loadOBJ("model.obj", vertices, triangles, normals)) {
        std::cerr << "Failed to load OBJ\n";
        return -1;
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
