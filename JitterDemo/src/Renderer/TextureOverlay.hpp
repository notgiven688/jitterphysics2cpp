class TexturedQuad
{
public:
    TexturedQuad()
        : ShaderProgram_(CreateShaderProgram(VertexShaderSource, FragmentShaderSource))
    {
        const std::array<Vec2, 4> vertices {{
            Vec2 {0.0f, 0.0f},
            Vec2 {1.0f, 0.0f},
            Vec2 {1.0f, 1.0f},
            Vec2 {0.0f, 1.0f},
        }};
        const std::array<unsigned int, 6> indices {{1, 0, 2, 2, 0, 3}};

        glGenVertexArrays(1, &VertexArray_);
        glGenBuffers(1, &VertexBuffer_);
        glGenBuffers(1, &ElementBuffer_);

        glBindVertexArray(VertexArray_);

        glBindBuffer(GL_ARRAY_BUFFER, VertexBuffer_);
        glBufferData(
            GL_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(vertices.size() * sizeof(Vec2)),
            vertices.data(),
            GL_STATIC_DRAW);

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ElementBuffer_);
        glBufferData(
            GL_ELEMENT_ARRAY_BUFFER,
            static_cast<GLsizeiptr>(indices.size() * sizeof(unsigned int)),
            indices.data(),
            GL_STATIC_DRAW);

        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, sizeof(Vec2), nullptr);

        ProjectionLocation_ = glGetUniformLocation(ShaderProgram_, "uProjection");
        OffsetLocation_ = glGetUniformLocation(ShaderProgram_, "uOffset");
        SizeLocation_ = glGetUniformLocation(ShaderProgram_, "uSize");
        TextureLocation_ = glGetUniformLocation(ShaderProgram_, "uTexture");

        glUseProgram(ShaderProgram_);
        glUniform1i(TextureLocation_, 0);
        glUseProgram(0);

        glBindVertexArray(0);
    }

    TexturedQuad(const TexturedQuad&) = delete;
    TexturedQuad& operator=(const TexturedQuad&) = delete;

    ~TexturedQuad()
    {
        if (ElementBuffer_ != 0)
        {
            glDeleteBuffers(1, &ElementBuffer_);
        }
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

    void Draw(const Texture2D& texture, int framebufferWidth, int framebufferHeight)
    {
        texture.Bind(GL_TEXTURE0);

        glBindVertexArray(VertexArray_);
        glUseProgram(ShaderProgram_);

        const Mat4 projection = OrthographicOffCenter(
            0.0f,
            static_cast<float>(framebufferWidth),
            static_cast<float>(framebufferHeight),
            0.0f,
            1.0f,
            -1.0f);
        glUniformMatrix4fv(ProjectionLocation_, 1, GL_FALSE, projection.Data());
        glUniform2f(OffsetLocation_, Position.X, Position.Y);
        glUniform2f(SizeLocation_, Size.X, Size.Y);

        glEnable(GL_BLEND);
        glDisable(GL_DEPTH_TEST);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
        glDisable(GL_BLEND);
        glEnable(GL_DEPTH_TEST);

        glUseProgram(0);
        glBindVertexArray(0);
    }

    Vec2 Position {0.0f, 0.0f};
    Vec2 Size {200.0f, 200.0f};

private:
    static constexpr const char* VertexShaderSource = R"GLSL(#version 330 core
layout(location = 0) in vec2 aPos;

uniform vec2 uOffset;
uniform vec2 uSize;
uniform mat4 uProjection;

out vec2 vUV;

void main()
{
    gl_Position = uProjection * vec4(aPos * uSize + uOffset, 0.0, 1.0);
    vUV = aPos;
}
)GLSL";

    static constexpr const char* FragmentShaderSource = R"GLSL(#version 330 core
uniform sampler2D uTexture;
in vec2 vUV;
out vec4 FragColor;

void main() { FragColor = texture(uTexture, vUV); }
)GLSL";

    GLuint VertexArray_ = 0;
    GLuint VertexBuffer_ = 0;
    GLuint ElementBuffer_ = 0;
    GLuint ShaderProgram_ = 0;
    GLint ProjectionLocation_ = -1;
    GLint OffsetLocation_ = -1;
    GLint SizeLocation_ = -1;
    GLint TextureLocation_ = -1;
};
