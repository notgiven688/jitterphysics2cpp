void Check(bool condition, const char* message)
{
    if (!condition)
    {
        throw std::runtime_error(message);
    }
}

Vec3 operator+(Vec3 left, Vec3 right)
{
    return Vec3 {left.X + right.X, left.Y + right.Y, left.Z + right.Z};
}

Vec3 operator-(Vec3 left, Vec3 right)
{
    return Vec3 {left.X - right.X, left.Y - right.Y, left.Z - right.Z};
}

Vec3 operator*(Vec3 value, float scale)
{
    return Vec3 {value.X * scale, value.Y * scale, value.Z * scale};
}

float Dot(Vec3 left, Vec3 right)
{
    return left.X * right.X + left.Y * right.Y + left.Z * right.Z;
}

Vec3 Cross(Vec3 left, Vec3 right)
{
    return Vec3 {
        left.Y * right.Z - left.Z * right.Y,
        left.Z * right.X - left.X * right.Z,
        left.X * right.Y - left.Y * right.X,
    };
}

float Length(Vec3 value)
{
    return std::sqrt(Dot(value, value));
}

Vec3 Normalize(Vec3 value)
{
    const float length = Length(value);
    if (length <= 0.0f)
    {
        return Vec3 {};
    }

    return value * (1.0f / length);
}

Vec4 operator*(Vec4 value, float scale)
{
    return Vec4 {value.X * scale, value.Y * scale, value.Z * scale, value.W * scale};
}

Vec3 FromJitter(const JVector& value)
{
    return Vec3 {
        static_cast<float>(value.X),
        static_cast<float>(value.Y),
        static_cast<float>(value.Z),
    };
}

JVector ToJitter(const Vec3& value)
{
    return JVector(
        static_cast<Jitter2::Real>(value.X),
        static_cast<Jitter2::Real>(value.Y),
        static_cast<Jitter2::Real>(value.Z));
}

Mat4 Identity()
{
    Mat4 result {};
    result.Values[0] = 1.0f;
    result.Values[5] = 1.0f;
    result.Values[10] = 1.0f;
    result.Values[15] = 1.0f;
    return result;
}

Mat4 Multiply(const Mat4& left, const Mat4& right)
{
    Mat4 result {};
    for (int column = 0; column < 4; ++column)
    {
        for (int row = 0; row < 4; ++row)
        {
            float value = 0.0f;
            for (int k = 0; k < 4; ++k)
            {
                value += left.Values[static_cast<std::size_t>(k * 4 + row)]
                    * right.Values[static_cast<std::size_t>(column * 4 + k)];
            }
            result.Values[static_cast<std::size_t>(column * 4 + row)] = value;
        }
    }
    return result;
}

Vec4 Transform(const Mat4& matrix, Vec4 value)
{
    return Vec4 {
        matrix.Values[0] * value.X + matrix.Values[4] * value.Y + matrix.Values[8] * value.Z + matrix.Values[12] * value.W,
        matrix.Values[1] * value.X + matrix.Values[5] * value.Y + matrix.Values[9] * value.Z + matrix.Values[13] * value.W,
        matrix.Values[2] * value.X + matrix.Values[6] * value.Y + matrix.Values[10] * value.Z + matrix.Values[14] * value.W,
        matrix.Values[3] * value.X + matrix.Values[7] * value.Y + matrix.Values[11] * value.Z + matrix.Values[15] * value.W,
    };
}

bool Invert(const Mat4& matrix, Mat4& result)
{
    const float* m = matrix.Values.data();
    std::array<float, 16> inv {};

    inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15]
        + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15]
        - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15]
        + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
    inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14]
        - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];

    inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15]
        - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15]
        + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15]
        - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
    inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14]
        + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];

    inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15]
        + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
    inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15]
        - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
    inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15]
        + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
    inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14]
        - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];

    inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11]
        - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
    inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11]
        + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
    inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11]
        - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
    inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10]
        + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];

    float determinant = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
    if (determinant == 0.0f)
    {
        return false;
    }

    determinant = 1.0f / determinant;
    for (std::size_t i = 0; i < inv.size(); ++i)
    {
        result.Values[i] = inv[i] * determinant;
    }
    return true;
}

Mat4 Perspective(float verticalFovRadians, float aspect, float nearPlane, float farPlane)
{
    const float focalLength = 1.0f / std::tan(verticalFovRadians * 0.5f);

    Mat4 result {};
    result.Values[0] = focalLength / aspect;
    result.Values[5] = focalLength;
    result.Values[10] = farPlane / (nearPlane - farPlane);
    result.Values[11] = -1.0f;
    result.Values[14] = nearPlane * farPlane / (nearPlane - farPlane);
    return result;
}

Mat4 OrthographicOffCenter(float left, float right, float bottom, float top, float nearPlane, float farPlane)
{
    Mat4 result {};
    result.Values[0] = 2.0f / (right - left);
    result.Values[5] = 2.0f / (top - bottom);
    result.Values[10] = 1.0f / (nearPlane - farPlane);
    result.Values[12] = (left + right) / (left - right);
    result.Values[13] = (top + bottom) / (bottom - top);
    result.Values[14] = nearPlane / (nearPlane - farPlane);
    result.Values[15] = 1.0f;
    return result;
}

Mat4 LookAt(Vec3 eye, Vec3 center, Vec3 up)
{
    const Vec3 forward = Normalize(center - eye);
    const Vec3 side = Normalize(Cross(forward, up));
    const Vec3 cameraUp = Cross(side, forward);

    Mat4 result = Identity();
    result.Values[0] = side.X;
    result.Values[1] = cameraUp.X;
    result.Values[2] = -forward.X;

    result.Values[4] = side.Y;
    result.Values[5] = cameraUp.Y;
    result.Values[6] = -forward.Y;

    result.Values[8] = side.Z;
    result.Values[9] = cameraUp.Z;
    result.Values[10] = -forward.Z;

    result.Values[12] = -Dot(side, eye);
    result.Values[13] = -Dot(cameraUp, eye);
    result.Values[14] = Dot(forward, eye);
    return result;
}

Mat4 FromJitter(const Jitter2::LinearMath::JMatrix& matrix)
{
    Mat4 result = Identity();
    result.Values[0] = static_cast<float>(matrix.M11);
    result.Values[4] = static_cast<float>(matrix.M12);
    result.Values[8] = static_cast<float>(matrix.M13);

    result.Values[1] = static_cast<float>(matrix.M21);
    result.Values[5] = static_cast<float>(matrix.M22);
    result.Values[9] = static_cast<float>(matrix.M23);

    result.Values[2] = static_cast<float>(matrix.M31);
    result.Values[6] = static_cast<float>(matrix.M32);
    result.Values[10] = static_cast<float>(matrix.M33);
    return result;
}

Mat4 RotationX(float radians)
{
    Mat4 result = Identity();
    const float c = std::cos(radians);
    const float s = std::sin(radians);

    result.Values[5] = c;
    result.Values[9] = -s;
    result.Values[6] = s;
    result.Values[10] = c;
    return result;
}

Mat4 RotationY(float radians)
{
    Mat4 result = Identity();
    const float c = std::cos(radians);
    const float s = std::sin(radians);

    result.Values[0] = c;
    result.Values[8] = s;
    result.Values[2] = -s;
    result.Values[10] = c;
    return result;
}

Mat4 Scale(float scale)
{
    Mat4 result = Identity();
    result.Values[0] = scale;
    result.Values[5] = scale;
    result.Values[10] = scale;
    return result;
}

Mat4 Scale(const JVector& scale)
{
    Mat4 result = Identity();
    result.Values[0] = static_cast<float>(scale.X);
    result.Values[5] = static_cast<float>(scale.Y);
    result.Values[10] = static_cast<float>(scale.Z);
    return result;
}

Mat4 BodyTransform(const JVector& position, const JQuaternion& orientation)
{
    Mat4 result = FromJitter(Jitter2::LinearMath::JMatrix::CreateFromQuaternion(orientation));
    result.Values[12] = static_cast<float>(position.X);
    result.Values[13] = static_cast<float>(position.Y);
    result.Values[14] = static_cast<float>(position.Z);
    return result;
}

Mat4 Translation(const JVector& position)
{
    Mat4 result = Identity();
    result.Values[12] = static_cast<float>(position.X);
    result.Values[13] = static_cast<float>(position.Y);
    result.Values[14] = static_cast<float>(position.Z);
    return result;
}

Jitter2::LinearMath::JMatrix ScaleMatrix(const JVector& scale)
{
    return Jitter2::LinearMath::JMatrix(
        scale.X, 0, 0,
        0, scale.Y, 0,
        0, 0, scale.Z);
}

GLuint CompileShader(GLenum shaderType, const char* source)
{
    const GLuint shader = glCreateShader(shaderType);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    GLint success = GL_FALSE;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (success != GL_TRUE)
    {
        std::array<GLchar, 2048> log {};
        glGetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), nullptr, log.data());
        glDeleteShader(shader);
        throw std::runtime_error(std::string("OpenGL shader compilation failed: ") + log.data());
    }

    return shader;
}

GLuint CreateShaderProgram(const char* vertexSource, const char* fragmentSource)
{
    const GLuint vertexShader = CompileShader(GL_VERTEX_SHADER, vertexSource);
    const GLuint fragmentShader = CompileShader(GL_FRAGMENT_SHADER, fragmentSource);
    const GLuint program = glCreateProgram();
    glAttachShader(program, vertexShader);
    glAttachShader(program, fragmentShader);
    glLinkProgram(program);

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    GLint success = GL_FALSE;
    glGetProgramiv(program, GL_LINK_STATUS, &success);
    if (success != GL_TRUE)
    {
        std::array<GLchar, 2048> log {};
        glGetProgramInfoLog(program, static_cast<GLsizei>(log.size()), nullptr, log.data());
        glDeleteProgram(program);
        throw std::runtime_error(std::string("OpenGL shader link failed: ") + log.data());
    }

    return program;
}
