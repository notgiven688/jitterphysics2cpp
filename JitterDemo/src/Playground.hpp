#include "Demos/IDemo.hpp"

class DemoScene
{
public:
    DemoScene()
    {
        World.NullBody().Tag = RigidBodyTag {};
        demos = CreateDemos();
        ResetScene();
        AddFloor();
    }

    ~DemoScene()
    {
        CleanUpCurrentDemo();
        ClearSoftBodyDemos();
        World.Clear();
    }

    void Reset(int demoIndex = -1)
    {
        if (demoIndex >= 0)
        {
            SwitchDemo(demoIndex);
            return;
        }

        if (CurrentDemo >= 0)
        {
            SwitchDemo(CurrentDemo);
            return;
        }

        CleanUpCurrentDemo();
        ResetScene();
        AddFloor();
    }

    void SwitchDemo(int index)
    {
        if (demos.empty())
        {
            return;
        }

        CleanUpCurrentDemo();
        ResetScene();
        CurrentDemo = std::clamp(index, 0, static_cast<int>(demos.size()) - 1);
        currentDemo = demos[static_cast<std::size_t>(CurrentDemo)].get();
        currentDemo->Build(*this, World);
    }

    [[nodiscard]] const std::vector<std::unique_ptr<IDemo>>& Demos() const
    {
        return demos;
    }

    [[nodiscard]] IDemo* CurrentDemoInstance() const
    {
        return currentDemo;
    }

    [[nodiscard]] const Shapes::RigidBodyShape* FloorShapeForRendering() const
    {
        return FloorShape;
    }

    void ShootPrimitive(Vec3 cameraPosition, Vec3 cameraDirection)
    {
        constexpr Jitter2::Real primitiveVelocity = static_cast<Jitter2::Real>(20);

        Jitter2::RigidBody& body = World.CreateRigidBody();
        body.Position(ToJitter(cameraPosition));
        body.Velocity(ToJitter(cameraDirection * static_cast<float>(primitiveVelocity)));
        body.AddShape(CreateShape<Shapes::BoxShape>(static_cast<Jitter2::Real>(1)));
    }

    void UpdatePicking(GLFWwindow* window, const CameraState& camera, const ImGuiIO& io)
    {
        const bool leftMouseDown =
            window != nullptr && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;

        if (!io.WantCaptureMouse && ((leftMouseDown && !grabMouseWasDown) || grabbing))
        {
            double mouseX = 0.0;
            double mouseY = 0.0;
            glfwGetCursorPos(window, &mouseX, &mouseY);

            int width = 1;
            int height = 1;
            glfwGetWindowSize(window, &width, &height);

            Pick(camera, mouseX, mouseY, width, height, io.MouseWheel);
        }

        if (!leftMouseDown)
        {
            ClearGrab();
        }

        grabMouseWasDown = leftMouseDown;
    }

private:
    std::vector<std::unique_ptr<IDemo>> demos;
    IDemo* currentDemo = nullptr;

    void CleanUpCurrentDemo()
    {
        if (ICleanDemo* cleanDemo = dynamic_cast<ICleanDemo*>(currentDemo))
        {
            cleanDemo->CleanUp(*this);
        }
        currentDemo = nullptr;
    }

    void ResetScene()
    {
        ClearGrab();
        World.BroadPhaseFilter = nullptr;
        World.NarrowPhaseFilter = nullptr;
        Tracks.clear();
        ConveyorPlanks.clear();
        GearCouplings.clear();
        ClearSoftBodyDemos();
        ConveyorPhysicsTime = 0.0;
        World.Clear();
        ConstraintCarHinges.clear();
        ConstraintCarInstance.reset();
        RayCastCarInstance.reset();
        PlayerInstance.reset();
        World.DynamicTree().Filter(Jitter2::World::DefaultDynamicTreeFilter);
        World.BroadPhaseFilter = nullptr;
        World.ResetNarrowPhaseFilter();
        World.PreStep.Clear();
        World.PostStep.Clear();
        World.PreSubStep.Clear();
        ConveyorPreSubStepToken = 0;
        World.PostSubStep.Clear();
        World.Gravity = JVector(0, static_cast<Jitter2::Real>(-9.81), 0);
        World.SubstepCount = 1;
        World.SolverIterations(8, 4);
        World.AllowDeactivation = true;
        IgnoreCollisionFilter.reset();
        GearCollisionFilter.reset();
        FloorShape = nullptr;
        RotatingBox = nullptr;
        DoublePendulumBody0 = nullptr;
        DoublePendulumBody1 = nullptr;
        OwnedShapes.clear();
        LevelMesh.reset();
    }

    void ClearGrab()
    {
        if (grabConstraint != nullptr
            && !grabConstraint->Handle().IsZero()
            && !grabConstraint->Body1().Handle().IsZero()
            && !grabConstraint->Body2().Handle().IsZero())
        {
            World.Remove(*grabConstraint);
        }

        grabBody = nullptr;
        grabConstraint = nullptr;
        grabbing = false;
        grabMouseWasDown = false;
    }

    void ClearGrabIfBody(const Jitter2::RigidBody& body)
    {
        if (grabBody == &body)
        {
            ClearGrab();
        }
    }

    static Vec3 RayTo(
        const CameraState& camera,
        double mouseX,
        double mouseY,
        int width,
        int height)
    {
        if (width <= 0 || height <= 0)
        {
            return camera.Direction;
        }

        const float x = static_cast<float>(mouseX) / static_cast<float>(width) * 2.0f - 1.0f;
        const float y = -static_cast<float>(mouseY) / static_cast<float>(height) * 2.0f + 1.0f;
        const float aspect = static_cast<float>(width) / static_cast<float>(height);
        const float tanHalfFov = std::tan(camera.FieldOfView * 0.5f);

        const Vec3 forward = Normalize(camera.Direction);
        const Vec3 right = Normalize(Cross(forward, Vec3 {0.0f, 1.0f, 0.0f}));
        const Vec3 up = Cross(right, forward);

        return Normalize(forward + right * (x * aspect * tanHalfFov) + up * (y * tanHalfFov));
    }

    void Pick(
        const CameraState& camera,
        double mouseX,
        double mouseY,
        int width,
        int height,
        float scrollDelta)
    {
        const JVector origin = ToJitter(camera.Position);
        const JVector direction = ToJitter(RayTo(camera, mouseX, mouseY, width, height));

        if (grabbing)
        {
            if (grabBody == nullptr || grabBody->Handle().IsZero()
                || grabConstraint == nullptr || grabConstraint->Handle().IsZero())
            {
                ClearGrab();
                return;
            }

            hitDistance += static_cast<Jitter2::Real>(scrollDelta);
            grabConstraint->Anchor2(origin + hitDistance * direction);
            grabBody->SetActivationState(true);
            grabBody->Velocity(grabBody->Velocity() * static_cast<Jitter2::Real>(0.98));
            grabBody->AngularVelocity(grabBody->AngularVelocity() * static_cast<Jitter2::Real>(0.98));
        }
        else
        {
            if (grabConstraint != nullptr)
            {
                ClearGrab();
            }
            grabBody = nullptr;

            Jitter2::Collision::DynamicTree::Proxy* grabShape = nullptr;
            JVector hitNormal;
            Jitter2::Real lambda = 0;
            const bool result = World.DynamicTree().RayCast(origin, direction, grabShape, hitNormal, lambda);
            if (!result)
            {
                return;
            }

            const JVector hitPoint = origin + lambda * direction;

            if (grabShape != nullptr)
            {
                if (auto* softShape = dynamic_cast<SoftBodies::SoftBodyShape*>(grabShape))
                {
                    grabBody = &softShape->GetClosest(hitPoint);
                }
                else if (auto* rigidShape = dynamic_cast<Shapes::RigidBodyShape*>(grabShape))
                {
                    grabBody = rigidShape->GetRigidBody();
                }
            }

            if (grabBody == nullptr || grabBody->MotionTypeValue() != Jitter2::MotionType::Dynamic)
            {
                return;
            }

            grabbing = true;
            hitDistance = lambda;
            grabConstraint = &World.CreateConstraint<Constraints::DistanceLimit>(*grabBody, World.NullBody());
            grabConstraint->Initialize(hitPoint, hitPoint);
            grabConstraint->Softness(static_cast<Jitter2::Real>(0.01));
            grabConstraint->Bias(static_cast<Jitter2::Real>(0.1));
        }
    }

public:
    int CurrentDemo = -1;

    enum class SweepCastKind
    {
        Sphere,
        Box,
        Capsule,
        Cylinder,
    };

    Jitter2::World World;
    std::unique_ptr<Shapes::TriangleMesh> LevelMesh;
    std::vector<std::unique_ptr<Shapes::RigidBodyShape>> OwnedShapes;
    std::vector<BodyTrack> Tracks;
    std::vector<ConveyorPlank> ConveyorPlanks;
    Jitter2::World::WorldStepFunction::Token ConveyorPreSubStepToken = 0;
    std::vector<std::unique_ptr<GearCoupling>> GearCouplings;
    std::unique_ptr<ConstraintCar> ConstraintCarInstance;
    std::vector<std::unique_ptr<Constraints::HingeJoint>> ConstraintCarHinges;
    std::unique_ptr<RayCastCar> RayCastCarInstance;
    std::unique_ptr<Player> PlayerInstance;
    std::unique_ptr<InstancedDrawable> FloorRenderer;
    std::unique_ptr<Dust> LevelRenderer;
    std::unique_ptr<CarMesh> CarRenderer;
    std::unique_ptr<WheelMesh> WheelRenderer;
    std::unique_ptr<FractureFragmentsDrawable> FractureRenderer;
    std::unordered_map<Jitter2::RigidBody*, std::unique_ptr<Breakable>> Breakables;
    std::vector<Breakable*> PendingBreaks;
    std::vector<FractureFragment> FractureFragments;
    std::size_t FracturePostStepToken = 0;
    bool FracturePreviousB = false;
    Jitter2::RigidBody* LevelBody = nullptr;
    bool LevelDebugDraw = false;
    bool LevelPreviousO = false;
    std::vector<std::unique_ptr<SoftBodySphereDemo>> SoftBodySpheres;
    std::vector<std::unique_ptr<SoftBodyCubeDemo>> SoftBodyCubes;
    std::vector<std::unique_ptr<SoftBodyClothDemo>> SoftBodyCloths;
    std::unique_ptr<DecomposedTeapot> TeapotRenderer;
    std::unique_ptr<ConvexDecomposition> TeapotDecomp;
    std::unique_ptr<Teapot> PointCloudTeapotRenderer;
    std::vector<Jitter2::RigidBody*> PointCloudTeapotBodies;
    Mat4 PointCloudTeapotShift {};
    std::unique_ptr<MutableMeshDrawable> ClothRenderer;
    std::unique_ptr<SoftBodies::BroadPhaseCollisionFilter> SoftBroadPhaseFilter;
    std::unique_ptr<Shapes::ConeShape> PointTestShape;
    std::unique_ptr<Dragon> DragonRenderer;
    std::unique_ptr<Octree> DragonOctree;
    std::unique_ptr<Tester> OctreeTestShape;
    std::unique_ptr<CustomCollisionDetection> OctreeCollisionFilter;
    std::unique_ptr<VoxelWorld> VoxelProxy;
    std::unique_ptr<VoxelCollisionFilter> VoxelBroadPhaseFilter;
    std::unique_ptr<InstancedDrawable> VoxelRenderer;
    std::unique_ptr<HeightmapTester> HeightmapProxy;
    std::unique_ptr<HeightmapDetection> HeightmapBroadPhaseFilter;
    std::unique_ptr<MutableMeshDrawable> HeightmapRenderer;
    Shapes::BoxShape* AngularSweepStaticBar = nullptr;
    Shapes::BoxShape* AngularSweepDynamicBox = nullptr;
    JVector AngularSweepPosition {0, 0, 10};
    JVector AngularSweepVelocity {0, 0, -10};
    JVector AngularSweepAngularVelocity {1, 2, 2};
    std::unique_ptr<InstancedDrawable> AngularSweepCubeRenderer;
    SweepCastKind CurrentSweepCastKind = SweepCastKind::Sphere;
    bool SweepCastPreviousO = false;
    bool SweepCastPreviousP = false;
    std::unique_ptr<InstancedDrawable> SweepSphereRenderer;
    std::unique_ptr<InstancedDrawable> SweepBoxRenderer;
    std::unique_ptr<InstancedDrawable> SweepCylinderRenderer;
    std::unique_ptr<InstancedDrawable> SweepHalfSphereRenderer;
    std::unique_ptr<CcdSolver> CcdSolverInstance;
    std::unique_ptr<IgnoreCollisionBetweenFilter> IgnoreCollisionFilter;
    std::unique_ptr<IgnoreGearCollisionFilter> GearCollisionFilter;
    std::unordered_map<ChunkKey, std::vector<VoxelRenderData>, ChunkKeyHash> VoxelChunkCache;
    std::mutex VoxelChunkCacheMutex;
    std::vector<std::vector<VoxelRenderData>> VoxelListPool;
    std::mutex VoxelListPoolMutex;
    int VoxelFrameCount = 0;
    Shapes::RigidBodyShape* FloorShape = nullptr;
    Jitter2::RigidBody* RotatingBox = nullptr;
    Jitter2::RigidBody* DoublePendulumBody0 = nullptr;
    Jitter2::RigidBody* DoublePendulumBody1 = nullptr;
    Jitter2::RigidBody* grabBody = nullptr;
    Constraints::DistanceLimit* grabConstraint = nullptr;
    Jitter2::Real hitDistance = static_cast<Jitter2::Real>(0);
    bool grabbing = false;
    bool grabMouseWasDown = false;
    double ConveyorPhysicsTime = 0.0;

    void DrawUpdate(
        DebugRenderer& debugRenderer,
        Vec3 cameraPosition,
        Vec3 cameraDirection,
        GLFWwindow* window)
    {
        if (IDrawUpdate* drawUpdate = dynamic_cast<IDrawUpdate*>(currentDemo))
        {
            drawUpdate->DrawUpdate(*this, debugRenderer, cameraPosition, cameraDirection, window);
        }
    }

    void RenderMutableDrawables(Renderer& renderer, const CameraMatrices& cameraMatrices, Vec3 cameraPosition)
    {
        RenderMutableDrawables(renderer, cameraMatrices, cameraPosition, nullptr);
    }

    void QueueFrameDrawables()
    {
        if (FloorShape != nullptr)
        {
            EnsureFloorRenderer();
            FloorRenderer->Push(Identity());
        }
    }

    void RenderShadowDrawables(ShadowCaster& shadowCaster)
    {
        if (FloorRenderer != nullptr)
        {
            shadowCaster.DrawInstancedDrawable(*FloorRenderer);
        }
        if (ClothRenderer != nullptr)
        {
            shadowCaster.DrawMutableMeshDrawable(*ClothRenderer);
        }
        if (HeightmapRenderer != nullptr)
        {
            shadowCaster.DrawMutableMeshDrawable(*HeightmapRenderer);
        }
        if (FractureRenderer != nullptr)
        {
            shadowCaster.DrawMutableMeshDrawable(*FractureRenderer);
        }
        if (TeapotRenderer != nullptr)
        {
            shadowCaster.DrawInstancedDrawable(TeapotRenderer->Drawable);
        }
        if (LevelRenderer != nullptr)
        {
            shadowCaster.DrawInstancedDrawable(LevelRenderer->Drawable);
        }
        if (CarRenderer != nullptr)
        {
            shadowCaster.DrawInstancedDrawable(CarRenderer->Drawable);
        }
        if (WheelRenderer != nullptr)
        {
            shadowCaster.DrawInstancedDrawable(WheelRenderer->Drawable);
        }
        if (PointCloudTeapotRenderer != nullptr)
        {
            shadowCaster.DrawInstancedDrawable(PointCloudTeapotRenderer->Drawable);
        }
        if (DragonRenderer != nullptr)
        {
            shadowCaster.DrawInstancedDrawable(DragonRenderer->Drawable);
        }
        if (VoxelRenderer != nullptr)
        {
            shadowCaster.DrawInstancedDrawable(*VoxelRenderer);
        }
        if (AngularSweepCubeRenderer != nullptr)
        {
            shadowCaster.DrawInstancedDrawable(*AngularSweepCubeRenderer);
        }
        if (SweepSphereRenderer != nullptr)
        {
            shadowCaster.DrawInstancedDrawable(*SweepSphereRenderer);
        }
        if (SweepBoxRenderer != nullptr)
        {
            shadowCaster.DrawInstancedDrawable(*SweepBoxRenderer);
        }
        if (SweepCylinderRenderer != nullptr)
        {
            shadowCaster.DrawInstancedDrawable(*SweepCylinderRenderer);
        }
        if (SweepHalfSphereRenderer != nullptr)
        {
            shadowCaster.DrawInstancedDrawable(*SweepHalfSphereRenderer);
        }
    }

    void RenderMutableDrawables(
        Renderer& renderer,
        const CameraMatrices& cameraMatrices,
        Vec3 cameraPosition,
        const ShadowCaster* shadowCaster)
    {
        if (FloorRenderer != nullptr)
        {
            renderer.RenderInstancedDrawable(*FloorRenderer, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (ClothRenderer != nullptr)
        {
            renderer.RenderMutableMeshDrawable(*ClothRenderer, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (HeightmapRenderer != nullptr)
        {
            renderer.RenderMutableMeshDrawable(*HeightmapRenderer, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (FractureRenderer != nullptr)
        {
            renderer.RenderMutableMeshDrawable(*FractureRenderer, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (TeapotRenderer != nullptr)
        {
            renderer.RenderInstancedDrawable(TeapotRenderer->Drawable, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (LevelRenderer != nullptr)
        {
            renderer.RenderInstancedDrawable(LevelRenderer->Drawable, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (CarRenderer != nullptr)
        {
            renderer.RenderInstancedDrawable(CarRenderer->Drawable, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (WheelRenderer != nullptr)
        {
            renderer.RenderInstancedDrawable(WheelRenderer->Drawable, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (PointCloudTeapotRenderer != nullptr)
        {
            renderer.RenderInstancedDrawable(PointCloudTeapotRenderer->Drawable, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (DragonRenderer != nullptr)
        {
            renderer.RenderInstancedDrawable(DragonRenderer->Drawable, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (VoxelRenderer != nullptr)
        {
            renderer.RenderInstancedDrawable(*VoxelRenderer, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (AngularSweepCubeRenderer != nullptr)
        {
            renderer.RenderInstancedDrawable(*AngularSweepCubeRenderer, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (SweepSphereRenderer != nullptr)
        {
            renderer.RenderInstancedDrawable(*SweepSphereRenderer, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (SweepBoxRenderer != nullptr)
        {
            renderer.RenderInstancedDrawable(*SweepBoxRenderer, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (SweepCylinderRenderer != nullptr)
        {
            renderer.RenderInstancedDrawable(*SweepCylinderRenderer, cameraMatrices, cameraPosition, shadowCaster);
        }
        if (SweepHalfSphereRenderer != nullptr)
        {
            renderer.RenderInstancedDrawable(*SweepHalfSphereRenderer, cameraMatrices, cameraPosition, shadowCaster);
        }
    }

    void DebugDraw(DebugRenderer& debugRenderer, const DemoSettings& settings)
    {
        if (settings.DebugDrawTree)
        {
            World.DynamicTree().EnumerateTreeBoxes(
                [&debugRenderer, &settings](const Jitter2::Collision::TreeBox& box, int depth)
                {
                    if (depth == settings.DebugDrawTreeDepth)
                    {
                        debugRenderer.PushBox(
                            DebugRenderer::Color::Green,
                            FromJitter(box.Min),
                            FromJitter(box.Max));
                    }
                });
        }

        if (settings.DebugDrawShapes)
        {
            for (const Jitter2::Collision::DynamicTree::Proxy* proxy : World.DynamicTree().Proxies())
            {
                const Jitter2::LinearMath::JBoundingBox& box = proxy->WorldBoundingBox();
                debugRenderer.PushBox(DebugRenderer::Color::Green, FromJitter(box.Min), FromJitter(box.Max));
            }
        }

        if (settings.DebugDrawIslands)
        {
            for (const Jitter2::Collision::Island* island : World.Islands().Elements())
            {
                bool active = false;
                Jitter2::LinearMath::JBoundingBox box = Jitter2::LinearMath::JBoundingBox::SmallBox();

                for (const Jitter2::RigidBody* body : island->Bodies())
                {
                    active = body->IsActive();
                    if (body->Shapes().empty())
                    {
                        Jitter2::LinearMath::JBoundingBox::AddPointInPlace(box, body->Position());
                    }
                    else
                    {
                        for (const Shapes::RigidBodyShape* shape : body->Shapes())
                        {
                            box = Jitter2::LinearMath::JBoundingBox::CreateMerged(box, shape->WorldBoundingBox());
                        }
                    }
                }

                debugRenderer.PushBox(
                    active ? DebugRenderer::Color::Green : DebugRenderer::Color::Red,
                    FromJitter(box.Min),
                    FromJitter(box.Max));
            }
        }

        if (settings.DebugDrawContacts)
        {
            const auto drawContact =
                [&debugRenderer](
                    const Jitter2::ContactData& contactData,
                    const Jitter2::ContactData::Contact& contact)
                {
                    const JVector point1 = contact.RelativePosition1 + contactData.Body1.Data().Position;
                    const JVector point2 = contact.RelativePosition2 + contactData.Body2.Data().Position;

                    debugRenderer.PushPoint(DebugRenderer::Color::Green, FromJitter(point1), 0.1f);
                    debugRenderer.PushPoint(DebugRenderer::Color::White, FromJitter(point2), 0.1f);
                };

            for (const Jitter2::ContactData& contactData : World.ActiveContacts())
            {
                const unsigned int usageMask = contactData.UsageMask >> 4U;
                if ((usageMask & Jitter2::ContactData::MaskContact0) != 0) drawContact(contactData, contactData.Contacts[0]);
                if ((usageMask & Jitter2::ContactData::MaskContact1) != 0) drawContact(contactData, contactData.Contacts[1]);
                if ((usageMask & Jitter2::ContactData::MaskContact2) != 0) drawContact(contactData, contactData.Contacts[2]);
                if ((usageMask & Jitter2::ContactData::MaskContact3) != 0) drawContact(contactData, contactData.Contacts[3]);
            }
        }
    }

private:
    void EnsureFloorRenderer()
    {
        if (FloorRenderer == nullptr)
        {
            FloorRenderer = std::make_unique<InstancedDrawable>(CreateQuadMesh(100.0f, 100.0f));
            FloorRenderer->MaterialValue = CreateFloorMaterial();
        }
    }

    void UpdateClothRenderVertices(SoftBodyClothDemo& cloth)
    {
        for (std::size_t i = 0; i < cloth.Vertices().size(); ++i)
        {
            ClothRenderer->Mesh.Vertices[i].Position = FromJitter(cloth.Vertices()[i]->Position());
        }

        ClothRenderer->RefreshGeometry();
    }

    void SetClothUVCoordinates(SoftBodyClothDemo& cloth)
    {
        for (std::size_t i = 0; i < cloth.Vertices().size(); ++i)
        {
            const JVector& position = cloth.Vertices()[i]->Data().Position;
            ClothRenderer->Mesh.Vertices[i].Texture = Vec2 {
                static_cast<float>(position.X),
                static_cast<float>(position.Z),
            };
        }
    }

    void ClearSoftBodyDemos()
    {
        for (const std::unique_ptr<SoftBodySphereDemo>& sphere : SoftBodySpheres)
        {
            sphere->Destroy();
        }
        SoftBodySpheres.clear();

        for (const std::unique_ptr<SoftBodyCubeDemo>& cube : SoftBodyCubes)
        {
            cube->Destroy();
        }
        SoftBodyCubes.clear();

        for (const std::unique_ptr<SoftBodyClothDemo>& cloth : SoftBodyCloths)
        {
            cloth->Destroy();
        }
        SoftBodyCloths.clear();

        CleanUpRealtimeFracture();
        LevelRenderer.reset();
        LevelBody = nullptr;
        LevelDebugDraw = false;
        LevelPreviousO = false;
        CarRenderer.reset();
        WheelRenderer.reset();
        ClothRenderer.reset();
        SoftBroadPhaseFilter.reset();
        PointTestShape.reset();
        if (TeapotDecomp != nullptr)
        {
            TeapotDecomp->Clear();
        }
        TeapotDecomp.reset();
        TeapotRenderer.reset();
        PointCloudTeapotRenderer.reset();
        PointCloudTeapotBodies.clear();
        PointCloudTeapotShift = Mat4 {};
        OctreeCollisionFilter.reset();
        if (OctreeTestShape != nullptr
            && OctreeTestShape->NodePtr() != Jitter2::Collision::DynamicTree::NullNode)
        {
            World.DynamicTree().RemoveProxy(*OctreeTestShape);
        }
        OctreeTestShape.reset();
        DragonOctree.reset();
        DragonRenderer.reset();
        VoxelBroadPhaseFilter.reset();
        if (VoxelProxy != nullptr
            && VoxelProxy->NodePtr() != Jitter2::Collision::DynamicTree::NullNode)
        {
            World.DynamicTree().RemoveProxy(*VoxelProxy);
        }
        VoxelProxy.reset();
        VoxelRenderer.reset();
        HeightmapBroadPhaseFilter.reset();
        if (HeightmapProxy != nullptr
            && HeightmapProxy->NodePtr() != Jitter2::Collision::DynamicTree::NullNode)
        {
            World.DynamicTree().RemoveProxy(*HeightmapProxy);
        }
        HeightmapProxy.reset();
        HeightmapRenderer.reset();
        AngularSweepStaticBar = nullptr;
        AngularSweepDynamicBox = nullptr;
        AngularSweepCubeRenderer.reset();
        SweepSphereRenderer.reset();
        SweepBoxRenderer.reset();
        SweepCylinderRenderer.reset();
        SweepHalfSphereRenderer.reset();
        CurrentSweepCastKind = SweepCastKind::Sphere;
        SweepCastPreviousO = false;
        SweepCastPreviousP = false;
        CcdSolverInstance.reset();
        {
            std::scoped_lock lock(VoxelChunkCacheMutex, VoxelListPoolMutex);
            VoxelChunkCache.clear();
            VoxelListPool.clear();
        }
        VoxelFrameCount = 0;
    }

    void ConfigureSoftBodyCollision()
    {
        World.DynamicTree().Filter(SoftBodies::DynamicTreeCollisionFilter::Filter);
        SoftBroadPhaseFilter = std::make_unique<SoftBodies::BroadPhaseCollisionFilter>(World);
        World.BroadPhaseFilter = SoftBroadPhaseFilter.get();
    }

#include "Demos/Demo00.hpp"
#include "Demos/Demo01.hpp"
#include "Demos/Demo02.hpp"
#include "Demos/Demo03.hpp"
#include "Demos/Demo04.hpp"
#include "Demos/Demo05.hpp"
#include "Demos/Demo06.hpp"
#include "Demos/Demo07.hpp"
#include "Demos/Demo08.hpp"
#include "Demos/Demo09.hpp"
#include "Demos/Demo10.hpp"
#include "Demos/Demo11.hpp"
#include "Demos/Demo12.hpp"
#include "Demos/Demo13.hpp"
#include "Demos/Demo14.hpp"
#include "Demos/Demo15.hpp"
#include "Demos/Demo16.hpp"
#include "Demos/Demo17.hpp"
#include "Demos/Demo18.hpp"
#include "Demos/Demo19.hpp"
#include "Demos/Demo20.hpp"
#include "Demos/Demo21.hpp"
#include "Demos/Demo22.hpp"
#include "Demos/Demo23.hpp"
#include "Demos/Demo24.hpp"
#include "Demos/Demo25.hpp"
#include "Demos/Demo26.hpp"
#include "Demos/Demo27.hpp"
#include "Demos/Demo28.hpp"
#include "Demos/Demo29.hpp"
#include "Demos/Demo30.hpp"
#include "Demos/Demo31.hpp"
#include "Demos/DemoJenga.hpp"
#include "Demos/DemoShowcase.hpp"

    void DrawUpdateDemo00(DebugRenderer&, Vec3, Vec3, GLFWwindow*)
    {
        if (TeapotDecomp != nullptr)
        {
            TeapotDecomp->PushMatrices();
        }
    }

    void DrawUpdateDemo01(DebugRenderer&, Vec3, Vec3, GLFWwindow* window)
    {
        DrawConstraintCar(window);
    }

    void DrawUpdateDemo05(DebugRenderer& debugRenderer, Vec3, Vec3, GLFWwindow* window)
    {
        DrawLevelGeometry(debugRenderer, window);
    }

    void DrawUpdateDemo06(DebugRenderer& debugRenderer, Vec3, Vec3, GLFWwindow* window)
    {
        DrawRayCastCar(debugRenderer, window);
    }

    void DrawUpdateDemo11(DebugRenderer& debugRenderer, Vec3, Vec3, GLFWwindow*)
    {
        DrawDoublePendulum(debugRenderer);
    }

    void DrawUpdateDemo15(DebugRenderer& debugRenderer, Vec3, Vec3, GLFWwindow*)
    {
        for (const std::unique_ptr<SoftBodySphereDemo>& sphere : SoftBodySpheres)
        {
            for (const Constraints::Constraint* spring : sphere->Springs())
            {
                debugRenderer.PushLine(
                    DebugRenderer::Color::Green,
                    FromJitter(spring->Body1().Position()),
                    FromJitter(spring->Body2().Position()));
            }
        }
    }

    void DrawUpdateDemo16(DebugRenderer& debugRenderer, Vec3, Vec3, GLFWwindow*)
    {
        for (const std::unique_ptr<SoftBodyCubeDemo>& cube : SoftBodyCubes)
        {
            for (const auto& edge : SoftBodyCubeDemo::Edges)
            {
                debugRenderer.PushLine(
                    DebugRenderer::Color::Green,
                    FromJitter(cube->Vertices()[static_cast<std::size_t>(edge.first)]->Position()),
                    FromJitter(cube->Vertices()[static_cast<std::size_t>(edge.second)]->Position()));

                debugRenderer.PushPoint(
                    DebugRenderer::Color::White,
                    FromJitter(cube->Center().Position()),
                    0.2f);
            }
        }
    }

    void DrawUpdateDemo17(DebugRenderer&, Vec3, Vec3, GLFWwindow*)
    {
        if (!SoftBodyCloths.empty() && ClothRenderer != nullptr)
        {
            SoftBodyClothDemo& cloth = *SoftBodyCloths.front();
            UpdateClothRenderVertices(cloth);
            ClothRenderer->Push(Identity(), Vec3 {0.0f, 1.0f, 0.0f});
        }
    }

    void DrawUpdateDemo20(DebugRenderer&, Vec3, Vec3, GLFWwindow*)
    {
        if (DragonRenderer != nullptr)
        {
            DragonRenderer->Push(Identity(), Vec3 {0.35f, 0.35f, 0.35f});
        }
    }

    void DrawUpdateDemo21(DebugRenderer&, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow*)
    {
        DrawVoxelWorld(cameraPosition, cameraDirection);
    }

    void DrawUpdateDemo22(DebugRenderer& debugRenderer, Vec3, Vec3, GLFWwindow*)
    {
        DrawConveyorBelt(debugRenderer);
    }

    void DrawUpdateDemo23(DebugRenderer&, Vec3, Vec3, GLFWwindow*)
    {
        if (RotatingBox != nullptr)
        {
            RotatingBox->AngularVelocity(JVector(
                static_cast<Jitter2::Real>(0.14),
                static_cast<Jitter2::Real>(0.02),
                static_cast<Jitter2::Real>(0.03)));
        }
    }

    void DrawUpdateDemo24(DebugRenderer&, Vec3, Vec3, GLFWwindow*)
    {
        if (PointCloudTeapotRenderer != nullptr)
        {
            DrawPointCloudTeapots();
        }
    }

    void DrawUpdateDemo25(DebugRenderer&, Vec3, Vec3, GLFWwindow*)
    {
        if (HeightmapRenderer != nullptr)
        {
            HeightmapRenderer->Push(Identity(), Vec3 {1.0f, 1.0f, 1.0f});
        }
    }

    void DrawUpdateDemo26(DebugRenderer& debugRenderer, Vec3, Vec3, GLFWwindow* window)
    {
        DrawAngularSweep(debugRenderer, window);
    }

    void DrawUpdateDemo29(DebugRenderer& debugRenderer, Vec3, Vec3, GLFWwindow*)
    {
        DrawGearCouplings(debugRenderer);
    }

    void DrawUpdateDemo30(DebugRenderer&, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window)
    {
        DrawSweepCasts(cameraPosition, cameraDirection, window);
    }

    void DrawUpdateDemo31(DebugRenderer&, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window)
    {
        DrawRealtimeFracture(cameraPosition, cameraDirection, window);
    }

    void DrawUpdatePointTest(DebugRenderer& debugRenderer, Vec3, Vec3, GLFWwindow*)
    {
        DrawPointTest(debugRenderer);
    }

    void DrawUpdateRayCastTest(DebugRenderer& debugRenderer, Vec3, Vec3, GLFWwindow*)
    {
        DrawRayCastTest(debugRenderer);
    }

    void CleanUpDemo15()
    {
        for (const std::unique_ptr<SoftBodySphereDemo>& sphere : SoftBodySpheres)
        {
            sphere->Destroy();
        }
        SoftBodySpheres.clear();
    }

    void CleanUpDemo16()
    {
        for (const std::unique_ptr<SoftBodyCubeDemo>& cube : SoftBodyCubes)
        {
            cube->Destroy();
        }
        SoftBodyCubes.clear();
    }

    void CleanUpDemo17()
    {
        for (const std::unique_ptr<SoftBodyClothDemo>& cloth : SoftBodyCloths)
        {
            cloth->Destroy();
        }
        SoftBodyCloths.clear();
        ClothRenderer.reset();
    }

    void CleanUpDemo20()
    {
        if (OctreeTestShape != nullptr
            && OctreeTestShape->NodePtr() != Jitter2::Collision::DynamicTree::NullNode)
        {
            World.DynamicTree().RemoveProxy(*OctreeTestShape);
        }
    }

    void CleanUpDemo21()
    {
        if (VoxelProxy != nullptr
            && VoxelProxy->NodePtr() != Jitter2::Collision::DynamicTree::NullNode)
        {
            World.DynamicTree().RemoveProxy(*VoxelProxy);
        }
    }

    void CleanUpDemo22()
    {
        if (ConveyorPreSubStepToken != 0)
        {
            World.PreSubStep.Remove(ConveyorPreSubStepToken);
            ConveyorPreSubStepToken = 0;
        }
        ConveyorPlanks.clear();
        ConveyorPhysicsTime = 0.0;
    }

    void CleanUpDemo27()
    {
        CcdSolverInstance.reset();
    }

    void CleanUpDemo29()
    {
        for (const std::unique_ptr<GearCoupling>& coupling : GearCouplings)
        {
            coupling->Remove();
        }
        GearCouplings.clear();
    }

    void CleanUpDemo31()
    {
        CleanUpRealtimeFracture();
    }

    class Demo00 final : public IDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildConvexDecompositionScene(); }
        [[nodiscard]] const char* Name() const override { return "Convex Decomposition"; }
        [[nodiscard]] const char* Description() const override { return "Convex-decomposed teapot models with compound convex hull collision."; }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo00(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo01 final : public IDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildConstraintCarScene(); }
        [[nodiscard]] const char* Name() const override { return "Constraint car"; }
        [[nodiscard]] const char* Description() const override { return "Constraint-based car with a breakable hinge-joint suspension bridge."; }
        [[nodiscard]] const char* Controls() const override { return "Arrow Keys - Steer and accelerate"; }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo01(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo02 final : public IDemo
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildTowerScene(); }
        [[nodiscard]] const char* Name() const override { return "Tower of Jitter"; }
        [[nodiscard]] const char* Description() const override { return "A single tower of stacked bodies to test solver stability."; }
    };

    class Demo03 final : public IDemo
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildPyramidScene(); }
        [[nodiscard]] const char* Name() const override { return "Ancient Pyramids"; }
        [[nodiscard]] const char* Description() const override { return "Large pyramids of boxes and cylinders to stress-test stacking."; }
    };

    class Demo04 final : public IDemo
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildManyRagdollsScene(); }
        [[nodiscard]] const char* Name() const override { return "Many Ragdolls"; }
        [[nodiscard]] const char* Description() const override { return "100 ragdolls dropping from increasing heights with collision filtering between limbs."; }
    };

    class Demo05 final : public IDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildLevelGeometryScene(); }
        [[nodiscard]] const char* Name() const override { return "Level Geometry"; }
        [[nodiscard]] const char* Description() const override { return "Triangle-mesh level loaded from an OBJ file with a player character."; }
        [[nodiscard]] const char* Controls() const override { return "Arrow Keys - Move player\nLeft Ctrl - Jump\nO - Toggle debug draw"; }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo05(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo06 final : public IDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildRayCastCarScene(); }
        [[nodiscard]] const char* Name() const override { return "Ray-cast Car"; }
        [[nodiscard]] const char* Description() const override { return "Drivable car using raycasts for wheel-ground contact."; }
        [[nodiscard]] const char* Controls() const override { return "Arrow Keys - Steer and accelerate"; }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo06(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo07 final : public IDemo
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildManyPyramidsScene(); }
        [[nodiscard]] const char* Name() const override { return "Many Pyramids"; }
        [[nodiscard]] const char* Description() const override { return "60 pre-deactivated box pyramids arranged in a grid."; }
    };

    class Demo08 final : public IDemo
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildContactManifoldTestScene(); }
        [[nodiscard]] const char* Name() const override { return "Contact Manifold Test"; }
    };

    class Demo09 final : public IDemo
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildRestitutionFrictionScene(); }
        [[nodiscard]] const char* Name() const override { return "Restitution and Friction"; }
        [[nodiscard]] const char* Description() const override { return "Varying restitution and friction values across rows of bodies."; }
    };

    class Demo10 final : public IDemo
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildStackedCubesScene(); }
        [[nodiscard]] const char* Name() const override { return "Stacked Cubes"; }
        [[nodiscard]] const char* Description() const override { return "Tall stacks of cubes and cones using sub-stepping for stability."; }
    };

    class Demo11 final : public IDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildDoublePendulumScene(); }
        [[nodiscard]] const char* Name() const override { return "Double Pendulum"; }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo11(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo12 final : public IDemo
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildSpeculativeContactsScene(); }
        [[nodiscard]] const char* Name() const override { return "Speculative Contacts"; }
        [[nodiscard]] const char* Description() const override { return "High-velocity objects fired at thin walls to test tunneling prevention."; }
    };

    class Demo13 final : public IDemo
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildMotorAndLimitScene(); }
        [[nodiscard]] const char* Name() const override { return "Motor and Limit"; }
        [[nodiscard]] const char* Description() const override { return "Hinge joints with angular motors, angular limits, and coupled rotating wheels."; }
    };

    class Demo14 final : public IDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildCustomShapesScene(); }
        [[nodiscard]] const char* Name() const override { return "Custom Shapes"; }
        [[nodiscard]] const char* Description() const override { return "Custom support-mapped shapes: ellipsoid, double-sphere, and icosahedron."; }
        void DrawUpdate(DemoScene&, DebugRenderer&, Vec3, Vec3, GLFWwindow*) override {}
    };

    class Demo15 final : public IDemo, public ICleanDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildPressurizedSoftBodiesScene(); }
        [[nodiscard]] const char* Name() const override { return "Pressurized Soft Bodies"; }
        [[nodiscard]] const char* Description() const override { return "Soft-body spheres using spring-connected particles and internal pressure."; }
        void CleanUp(DemoScene& pg) override { pg.CleanUpDemo15(); }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo15(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo16 final : public IDemo, public ICleanDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildSoftBodyCubesScene(); }
        [[nodiscard]] const char* Name() const override { return "Soft Body Cubes"; }
        [[nodiscard]] const char* Description() const override { return "Deformable soft-body cubes interacting with rigid bodies."; }
        void CleanUp(DemoScene& pg) override { pg.CleanUpDemo16(); }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo16(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo17 final : public IDemo, public ICleanDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildClothScene(); }
        [[nodiscard]] const char* Name() const override { return "Cloth"; }
        [[nodiscard]] const char* Description() const override { return "Cloth sheet pinned at its corners with rigid bodies falling onto it."; }
        void CleanUp(DemoScene& pg) override { pg.CleanUpDemo17(); }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo17(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo18 final : public IDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildPointTestScene(); }
        [[nodiscard]] const char* Name() const override { return "PointTest"; }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdatePointTest(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo19 final : public IDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildRayCastTestScene(); }
        [[nodiscard]] const char* Name() const override { return "RayCastTest"; }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateRayCastTest(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo20 final : public IDemo, public ICleanDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildOctreeScene(); }
        [[nodiscard]] const char* Name() const override { return "Octree (Custom Collision)"; }
        [[nodiscard]] const char* Description() const override { return "High-poly mesh in an octree with custom narrow-phase triangle collision."; }
        void CleanUp(DemoScene& pg) override { pg.CleanUpDemo20(); }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo20(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo21 final : public IDemo, public ICleanDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildVoxelWorldScene(); }
        [[nodiscard]] const char* Name() const override { return "Voxel World (Custom Collision)"; }
        [[nodiscard]] const char* Description() const override { return "Infinite procedural voxel terrain with custom collision shapes."; }
        void CleanUp(DemoScene& pg) override { pg.CleanUpDemo21(); }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo21(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo22 final : public IDemo, public ICleanDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildConveyorBeltScene(); }
        [[nodiscard]] const char* Name() const override { return "Conveyor Belt"; }
        [[nodiscard]] const char* Description() const override { return "Kinematic planks forming a conveyor belt that transports rigid bodies."; }
        void CleanUp(DemoScene& pg) override { pg.CleanUpDemo22(); }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo22(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo23 final : public IDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildRotatingCubeScene(); }
        [[nodiscard]] const char* Name() const override { return "Rotating Cube"; }
        [[nodiscard]] const char* Description() const override { return "Bodies inside a large kinematic rotating hollow cube."; }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo23(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo24 final : public IDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildPointCloudTeapotScene(); }
        [[nodiscard]] const char* Name() const override { return "Convex PointCloudShape"; }
        [[nodiscard]] const char* Description() const override { return "Convex hull collision shape created from sampled teapot vertices."; }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo24(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo25 final : public IDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildHeightmapScene(); }
        [[nodiscard]] const char* Name() const override { return "Heightmap (Custom Collision)"; }
        [[nodiscard]] const char* Description() const override { return "Procedural heightmap with custom per-triangle collision and raycasting."; }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo25(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo26 final : public IDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildAngularSweepScene(); }
        [[nodiscard]] const char* Name() const override { return "Angular Sweep"; }
        [[nodiscard]] const char* Description() const override
        {
            return "Visualizes NarrowPhase.Sweep with angular velocity. "
                   "A rotating box sweeps toward a static bar, showing "
                   "interpolated orientations up to the time of impact.";
        }
        [[nodiscard]] const char* Controls() const override { return "O/P - Move sweep origin forward/backward"; }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo26(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo27 final : public IDemo, public ICleanDemo
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildCcdScene(); }
        [[nodiscard]] const char* Name() const override { return "CCD: Proof of concept"; }
        [[nodiscard]] const char* Description() const override { return "Continuous collision detection with fast-moving objects and thin geometry."; }
        void CleanUp(DemoScene& pg) override { pg.CleanUpDemo27(); }
    };

    class Demo28 final : public IDemo
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildColosseumScene(); }
        [[nodiscard]] const char* Name() const override { return "Colosseum"; }
        [[nodiscard]] const char* Description() const override { return "Colosseum of concentric ring walls and platforms to benchmark large scenes."; }
    };

    class Demo29 final : public IDemo, public ICleanDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildGearsScene(); }
        [[nodiscard]] const char* Name() const override { return "Gears"; }
        [[nodiscard]] const char* Description() const override { return "Interlocking gear bodies coupled via constraints."; }
        void CleanUp(DemoScene& pg) override { pg.CleanUpDemo29(); }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo29(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo30 final : public IDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildSweepCastsScene(); }
        [[nodiscard]] const char* Name() const override { return "Sweep Casts"; }
        [[nodiscard]] const char* Description() const override
        {
            return "Camera-driven sweep casts through a simple physics scene. "
                   "Switch between sphere, box, capsule and cylinder queries "
                   "to compare the impact pose for the same view direction.";
        }
        [[nodiscard]] const char* Controls() const override { return "O/P - Previous/next cast type"; }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo30(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class Demo31 final : public IDemo, public ICleanDemo, public IDrawUpdate
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildRealtimeFractureScene(); }
        [[nodiscard]] const char* Name() const override { return "Realtime Fracture"; }
        [[nodiscard]] const char* Description() const override { return "Convex bodies fractured into smaller convex hulls at runtime."; }
        [[nodiscard]] const char* Controls() const override { return "B - Break aimed body"; }
        void CleanUp(DemoScene& pg) override { pg.CleanUpDemo31(); }
        void DrawUpdate(DemoScene& pg, DebugRenderer& debugRenderer, Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window) override
        {
            pg.DrawUpdateDemo31(debugRenderer, cameraPosition, cameraDirection, window);
        }
    };

    class DemoJenga final : public IDemo
    {
    public:
        void Build(DemoScene& pg, Jitter2::World&) override { pg.BuildJengaScene(); }
        [[nodiscard]] const char* Name() const override { return "Jenga"; }
    };

    std::vector<std::unique_ptr<IDemo>> CreateDemos()
    {
        std::vector<std::unique_ptr<IDemo>> result;
        result.reserve(32);
        result.push_back(std::make_unique<Demo00>());
        result.push_back(std::make_unique<Demo01>());
        result.push_back(std::make_unique<Demo02>());
        result.push_back(std::make_unique<Demo03>());
        result.push_back(std::make_unique<Demo04>());
        result.push_back(std::make_unique<Demo05>());
        result.push_back(std::make_unique<Demo06>());
        result.push_back(std::make_unique<Demo07>());
        // result.push_back(std::make_unique<Demo08>()); // contact manifold test
        result.push_back(std::make_unique<Demo09>());
        result.push_back(std::make_unique<Demo10>());
        // result.push_back(std::make_unique<Demo11>()); // double pendulum
        result.push_back(std::make_unique<Demo12>());
        result.push_back(std::make_unique<Demo13>());
        result.push_back(std::make_unique<Demo14>());
        result.push_back(std::make_unique<Demo15>());
        result.push_back(std::make_unique<Demo16>());
        result.push_back(std::make_unique<Demo17>());
        // result.push_back(std::make_unique<Demo18>()); // point test
        // result.push_back(std::make_unique<Demo19>()); // ray cast test
        result.push_back(std::make_unique<Demo20>());
        result.push_back(std::make_unique<Demo21>());
        result.push_back(std::make_unique<Demo22>());
        result.push_back(std::make_unique<Demo23>());
        result.push_back(std::make_unique<Demo24>());
        result.push_back(std::make_unique<Demo25>());
        result.push_back(std::make_unique<Demo26>()); // angular sweep
        result.push_back(std::make_unique<Demo27>());
        result.push_back(std::make_unique<Demo28>());
        result.push_back(std::make_unique<Demo29>());
        result.push_back(std::make_unique<Demo30>());
        result.push_back(std::make_unique<Demo31>());

        return result;
    }

public:
    void Animate(float speed)
    {
        const float t = static_cast<float>(World.Time()) * speed;
        for (BodyTrack& track : Tracks)
        {
            if (track.Body == nullptr)
            {
                continue;
            }

            const JVector offset(
                static_cast<Jitter2::Real>(std::cos(t * 0.74f + track.Phase) * track.Orbit),
                static_cast<Jitter2::Real>(std::sin(t * 1.18f + track.Phase) * track.Bob),
                static_cast<Jitter2::Real>(std::sin(t * 0.64f + track.Phase) * track.Orbit));
            track.Body->Position(track.BasePosition + offset);

            JQuaternion orientation = JQuaternion::CreateRotationY(
                static_cast<Jitter2::Real>(t * track.Spin + track.Phase))
                * JQuaternion::CreateRotationX(
                    static_cast<Jitter2::Real>(std::sin(t + track.Phase) * track.Tilt));
            orientation.Normalize();
            track.Body->Orientation(orientation);
        }
    }

private:
#include "Demos/CommonScene.hpp"
};
