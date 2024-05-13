```
// 初始化 GLFW
glfwInit();
glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

// 创建窗口
GLFWwindow* window = glfwCreateWindow(700, 700, "OpenGL Window", nullptr, nullptr);
if (window == nullptr) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return -1;
}
glfwMakeContextCurrent(window);

// 初始化 GLAD
if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return -1;
}

// 将 Eigen::Vector3f 转换为可以传递给 OpenGL 的格式
std::vector<unsigned char> imageData;
for (const auto& pixel : r.frame_buffer()) {
    imageData.push_back(static_cast<unsigned char>(pixel.x() * 255)); // R 值
    imageData.push_back(static_cast<unsigned char>(pixel.y() * 255)); // G 值
    imageData.push_back(static_cast<unsigned char>(pixel.z() * 255)); // B 值
}

// 创建并绑定 OpenGL 纹理
GLuint texture;
glGenTextures(1, &texture);
glBindTexture(GL_TEXTURE_2D, texture);
glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, 700, 700, 0, GL_RGB, GL_UNSIGNED_BYTE, imageData.data());
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

// 渲染循环
while (!glfwWindowShouldClose(window)) {
    // 清空缓冲区
    glClear(GL_COLOR_BUFFER_BIT);

    // 绘制纹理
    glBindTexture(GL_TEXTURE_2D, texture);
    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 0.0f); glVertex2f(-1.0f, -1.0f);
    glTexCoord2f(1.0f, 0.0f); glVertex2f(1.0f, -1.0f);
    glTexCoord2f(1.0f, 1.0f); glVertex2f(1.0f, 1.0f);
    glTexCoord2f(0.0f, 1.0f); glVertex2f(-1.0f, 1.0f);
    glEnd();

    // 交换缓冲区和事件处理
    glfwSwapBuffers(window);
    glfwPollEvents();
}

// 清理资源
glfwTerminate();
```