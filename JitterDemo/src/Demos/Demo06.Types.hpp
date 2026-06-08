std::shared_ptr<Texture2D> CarTexture()
{
    static std::shared_ptr<Texture2D> carTexture;
    if (carTexture == nullptr)
    {
        int width = 0;
        int height = 0;
        std::vector<unsigned char> pixels = LoadTgaBgra("assets/car.tga", width, height);
        carTexture = std::make_shared<Texture2D>();
        carTexture->LoadImage(pixels, width, height);
        carTexture->SetWrap(GL_REPEAT);
        carTexture->SetAnisotropy(8.0f);
    }

    return carTexture;
}

class WheelMesh : public TriangleMeshDrawable
{
public:
    WheelMesh()
        : TriangleMeshDrawable("wheel.obj")
    {
        Drawable.MaterialValue = Material {
            Vec3 {0.0f, 0.0f, 0.0f},
            Vec3 {1.0f, 1.0f, 1.0f},
            1000.0f,
            1.0f,
            0.05f,
            1.0f,
            false,
            CarTexture(),
        };
    }
};

class CarMesh : public TriangleMeshDrawable
{
public:
    CarMesh()
        : TriangleMeshDrawable("car.obj")
    {
        Drawable.MaterialValue = Material {
            Vec3 {0.0f, 0.0f, 0.0f},
            Vec3 {1.0f, 1.0f, 1.0f},
            1000.0f,
            1.0f,
            0.1f,
            1.2f,
            false,
            CarTexture(),
        };

        Material glass = Material {
            Vec3 {0.6f, 0.6f, 0.6f},
            Vec3 {1.0f, 1.0f, 1.0f},
            1000.0f,
            0.6f,
            0.0f,
            0.0f,
            false,
            nullptr,
        };

        Drawable.Groups = {
            MaterialSlot {0, glass},
        };
    }
};
