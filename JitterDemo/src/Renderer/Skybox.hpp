class Skybox
{
public:
    Skybox()
        : ShaderProgram_(CreateShaderProgram(VertexShaderSource, FragmentShaderSource))
    {
        glGenVertexArrays(1, &VertexArray_);
        glGenBuffers(1, &VertexBuffer_);

        glBindVertexArray(VertexArray_);
        glBindBuffer(GL_ARRAY_BUFFER, VertexBuffer_);
        glBufferData(
            GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(CubeVertices.size() * sizeof(float)),
            CubeVertices.data(),
            GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
        glBindVertexArray(0);

        ViewLocation_ = glGetUniformLocation(ShaderProgram_, "uView");
        ProjectionLocation_ = glGetUniformLocation(ShaderProgram_, "uProjection");
    }

    Skybox(const Skybox&) = delete;
    Skybox& operator=(const Skybox&) = delete;

    ~Skybox()
    {
        if (VertexBuffer_ != 0)
        {
            glDeleteBuffers(1, &VertexBuffer_);
        }
        if (VertexArray_ != 0)
        {
            glDeleteVertexArrays(1, &VertexArray_);
        }
        if (ShaderProgram_ != 0)
        {
            glDeleteProgram(ShaderProgram_);
        }
    }

    void Draw(const CameraMatrices& cameraMatrices) const
    {
        glDepthMask(GL_FALSE);
        glCullFace(GL_BACK);

        glUseProgram(ShaderProgram_);
        glUniformMatrix4fv(ViewLocation_, 1, GL_FALSE, cameraMatrices.View.Data());
        glUniformMatrix4fv(ProjectionLocation_, 1, GL_FALSE, cameraMatrices.Projection.Data());

        glBindVertexArray(VertexArray_);
        glDrawArrays(GL_TRIANGLES, 0, 36);
        glBindVertexArray(0);
        glUseProgram(0);

        glDepthMask(GL_TRUE);
    }

private:
    static constexpr std::array<float, 108> CubeVertices {{
        -1.0f,  1.0f, -1.0f, -1.0f, -1.0f, -1.0f,  1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f,  1.0f,  1.0f, -1.0f, -1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f, -1.0f, -1.0f, -1.0f, -1.0f,  1.0f, -1.0f,
        -1.0f,  1.0f, -1.0f, -1.0f,  1.0f,  1.0f, -1.0f, -1.0f,  1.0f,
         1.0f, -1.0f, -1.0f,  1.0f, -1.0f,  1.0f,  1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,  1.0f,  1.0f, -1.0f,  1.0f, -1.0f, -1.0f,
        -1.0f, -1.0f,  1.0f, -1.0f,  1.0f,  1.0f,  1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f,  1.0f, -1.0f,  1.0f, -1.0f, -1.0f,  1.0f,
        -1.0f,  1.0f, -1.0f,  1.0f,  1.0f, -1.0f,  1.0f,  1.0f,  1.0f,
         1.0f,  1.0f,  1.0f, -1.0f,  1.0f,  1.0f, -1.0f,  1.0f, -1.0f,
        -1.0f, -1.0f, -1.0f, -1.0f, -1.0f,  1.0f,  1.0f, -1.0f, -1.0f,
         1.0f, -1.0f, -1.0f, -1.0f, -1.0f,  1.0f,  1.0f, -1.0f,  1.0f,
    }};

    static constexpr const char* VertexShaderSource = R"GLSL(#version 330 core
layout(location = 0) in vec3 aPos;

out vec3 vDir;

uniform mat4 uProjection;
uniform mat4 uView;

void main()
{
    vDir = aPos;
    gl_Position = uProjection * mat4(mat3(uView)) * vec4(aPos, 1.0);
}
)GLSL";

    static constexpr const char* FragmentShaderSource = R"GLSL(#version 330 core
in vec3 vDir;
out vec4 FragColor;

void main()
{
    vec3 blue = vec3(66.0/255.0, 135.0/255.0, 245.0/255.0);
    float d = max(dot(vDir / length(vDir), vec3(0, 1, 1)) + 0.4, 0.0);
    FragColor = vec4(blue * 0.9 + vec3(1) * d * 0.1, 1.0);
}
)GLSL";

    GLuint ShaderProgram_ = 0;
    GLuint VertexArray_ = 0;
    GLuint VertexBuffer_ = 0;
    GLint ViewLocation_ = -1;
    GLint ProjectionLocation_ = -1;
};
