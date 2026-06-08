class Dust : public TriangleMeshDrawable
{
public:
    Dust()
        : TriangleMeshDrawable("level.obj", 0.8f)
    {
        Drawable.MaterialValue = Material {
            Vec3 {0.0f, 0.0f, 0.0f},
            Vec3 {0.1f, 0.1f, 0.1f},
            128.0f,
            1.0f,
            1.2f,
            0.5f,
            false,
            nullptr,
        };
    }
};
