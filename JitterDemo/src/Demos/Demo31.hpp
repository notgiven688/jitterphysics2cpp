// Member functions for DemoScene; included inside class DemoScene.

    void BuildRealtimeFractureScene()
    {
        FractureRenderer = std::make_unique<FractureFragmentsDrawable>();
        FractureRenderer->SetFragments(FractureFragments);

        AddFloor();
        World.SolverIterations(10, 5);
        World.AllowDeactivation = true;

        BuildBreakableWall(JVector(0, 0, -12));
        BuildBreakableSphere(
            JVector(static_cast<Jitter2::Real>(-4.05), static_cast<Jitter2::Real>(10.0), static_cast<Jitter2::Real>(0.0)),
            static_cast<Jitter2::Real>(0.95),
            4000);
        BuildBreakablePane(JVector(static_cast<Jitter2::Real>(0.0), static_cast<Jitter2::Real>(0.0), static_cast<Jitter2::Real>(-4.0)), 5000);

        FracturePostStepToken = World.PostStep.Add(
            [this](Jitter2::Real dt)
            {
                OnRealtimeFracturePostStep(dt);
            });
    }

    void CleanUpRealtimeFracture()
    {
        if (FracturePostStepToken != 0)
        {
            World.PostStep.Remove(FracturePostStepToken);
            FracturePostStepToken = 0;
        }

        for (auto& pair : Breakables)
        {
            if (pair.second != nullptr && pair.second->Body != nullptr)
            {
                pair.second->Body->BeginCollide = {};
            }
        }

        Breakables.clear();
        PendingBreaks.clear();
        FractureFragments.clear();
        if (FractureRenderer != nullptr)
        {
            FractureRenderer->SetFragments(FractureFragments);
        }
        FractureRenderer.reset();
        FracturePreviousB = false;
    }

    void DrawRealtimeFracture(Vec3 cameraPosition, Vec3 cameraDirection, GLFWwindow* window)
    {
        const bool bIsDown = window != nullptr && glfwGetKey(window, GLFW_KEY_B) == GLFW_PRESS;
        if (bIsDown && !FracturePreviousB)
        {
            QueueNearestBreakableToCamera(ToJitter(cameraPosition), ToJitter(cameraDirection));
        }
        FracturePreviousB = bIsDown;

        if (FractureRenderer != nullptr)
        {
            FractureRenderer->PushFragments();
        }
    }

    void BuildBreakableWall(JVector origin)
    {
        JVector blockSize(
            static_cast<Jitter2::Real>(1.7),
            static_cast<Jitter2::Real>(1.0),
            static_cast<Jitter2::Real>(0.9));
        constexpr int columns = 6;
        constexpr int rows = 5;
        int seed = 1000;

        for (int y = 0; y < rows; ++y)
        {
            Jitter2::Real rowOffset = (y & 1) == 0
                ? static_cast<Jitter2::Real>(0.0)
                : blockSize.X * static_cast<Jitter2::Real>(0.5);

            for (int x = 0; x < columns; ++x)
            {
                Jitter2::Real px = (static_cast<Jitter2::Real>(x)
                    - static_cast<Jitter2::Real>(columns - 1) * static_cast<Jitter2::Real>(0.5))
                    * blockSize.X + rowOffset;
                Jitter2::Real py = blockSize.Y * static_cast<Jitter2::Real>(0.5)
                    + static_cast<Jitter2::Real>(y) * blockSize.Y;

                RegisterBreakableBox(
                    origin + JVector(px, py, static_cast<Jitter2::Real>(0.0)),
                    blockSize,
                    seed++,
                    static_cast<Jitter2::Real>(0.55),
                    static_cast<Jitter2::Real>(0.04),
                    JQuaternion::Identity());
            }
        }
    }

    void BuildBreakablePane(JVector origin, int seed)
    {
        JVector paneSize(
            static_cast<Jitter2::Real>(4.4),
            static_cast<Jitter2::Real>(2.6),
            static_cast<Jitter2::Real>(0.24));
        RegisterBreakableBox(
            origin + JVector(static_cast<Jitter2::Real>(0.0), static_cast<Jitter2::Real>(7.5), static_cast<Jitter2::Real>(0.0)),
            paneSize,
            seed,
            static_cast<Jitter2::Real>(0.75),
            static_cast<Jitter2::Real>(0.03),
            JQuaternion::CreateRotationZ(static_cast<Jitter2::Real>(0.18))
                * JQuaternion::CreateRotationX(static_cast<Jitter2::Real>(0.08)));
    }

    void RegisterBreakableBox(
        JVector position,
        JVector size,
        int seed,
        Jitter2::Real friction,
        Jitter2::Real restitution,
        JQuaternion orientation)
    {
        Jitter2::RigidBody& body = World.CreateRigidBody();
        body.Position(position);
        body.Orientation(orientation);
        body.Friction(friction);
        body.Restitution(restitution);
        body.AddShape(CreateShape<Shapes::BoxShape>(size.X, size.Y, size.Z));
        body.Tag = BreakableBodyTag {};

        auto breakable = std::make_unique<Breakable>();
        breakable->Body = &body;
        breakable->Size = size;
        breakable->BoundsMin = size * static_cast<Jitter2::Real>(-0.5);
        breakable->BoundsMax = size * static_cast<Jitter2::Real>(0.5);
        breakable->Radius = size.Length() * static_cast<Jitter2::Real>(0.5);
        breakable->Seed = seed;
        breakable->Source = CreateBoxPolyhedron(size);

        body.BeginCollide =
            [this](Jitter2::Arbiter& arbiter)
            {
                OnBreakableCollide(arbiter);
            };
        Breakables.emplace(&body, std::move(breakable));
    }

    void BuildBreakableSphere(JVector position, Jitter2::Real radius, int seed)
    {
        Jitter2::RigidBody& body = World.CreateRigidBody();
        body.Position(position);
        body.Friction(static_cast<Jitter2::Real>(0.38));
        body.Restitution(static_cast<Jitter2::Real>(0.08));
        body.AddShape(CreateShape<Shapes::SphereShape>(radius));
        body.Tag = BreakableBodyTag {};

        auto breakable = std::make_unique<Breakable>();
        breakable->Body = &body;
        breakable->Size = JVector(radius * static_cast<Jitter2::Real>(2.0));
        breakable->BoundsMin = JVector(-radius);
        breakable->BoundsMax = JVector(radius);
        breakable->Radius = radius;
        breakable->Seed = seed;
        breakable->Source = CreateSpherePolyhedron(radius);
        breakable->SphericalSites = true;

        body.BeginCollide =
            [this](Jitter2::Arbiter& arbiter)
            {
                OnBreakableCollide(arbiter);
            };
        Breakables.emplace(&body, std::move(breakable));
    }

    void OnBreakableCollide(Jitter2::Arbiter& arbiter)
    {
        Jitter2::RigidBody* body = nullptr;
        Jitter2::RigidBody* other = nullptr;
        Breakable* breakable = nullptr;

        auto body1Iterator = Breakables.find(&arbiter.Body1());
        if (body1Iterator != Breakables.end())
        {
            body = &arbiter.Body1();
            other = &arbiter.Body2();
            breakable = body1Iterator->second.get();
        }
        else
        {
            auto body2Iterator = Breakables.find(&arbiter.Body2());
            if (body2Iterator != Breakables.end())
            {
                body = &arbiter.Body2();
                other = &arbiter.Body1();
                breakable = body2Iterator->second.get();
            }
        }

        if (body == nullptr || other == nullptr || breakable == nullptr)
        {
            return;
        }
        if (breakable->Generation > 0 && breakable->TimeSinceCreated < RefractureArmDelay)
        {
            return;
        }

        JVector relativeVelocity = body->Velocity() - other->Velocity();
        if (relativeVelocity.LengthSquared() < BreakSpeed * BreakSpeed)
        {
            return;
        }

        JVector impactPoint = EstimateImpactPoint(arbiter);
        JVector impactDirection = NormalizeSafe(body->Position() - other->Position());
        if (impactDirection.LengthSquared() < static_cast<Jitter2::Real>(0.5))
        {
            impactDirection = JVector::UnitY();
        }

        QueueBreak(*breakable, impactPoint, impactDirection);
    }

    static JVector EstimateImpactPoint(Jitter2::Arbiter& arbiter)
    {
        Jitter2::ContactData& data = arbiter.Data();
        unsigned int mask = data.UsageMask;

        JVector sum = JVector::Zero();
        int count = 0;

        const auto addContact =
            [&sum, &count, &data, &arbiter, mask](unsigned int contactMask, const Jitter2::ContactData::Contact& contact)
            {
                if ((mask & contactMask) == 0U)
                {
                    return;
                }

                JVector p1 = arbiter.Body1().Position() + contact.RelativePosition1;
                JVector p2 = arbiter.Body2().Position() + contact.RelativePosition2;
                sum += (p1 + p2) * static_cast<Jitter2::Real>(0.5);
                ++count;
            };

        addContact(Jitter2::ContactData::MaskContact0, data.Contacts[0]);
        addContact(Jitter2::ContactData::MaskContact1, data.Contacts[1]);
        addContact(Jitter2::ContactData::MaskContact2, data.Contacts[2]);
        addContact(Jitter2::ContactData::MaskContact3, data.Contacts[3]);

        return count == 0
            ? (arbiter.Body1().Position() + arbiter.Body2().Position()) * static_cast<Jitter2::Real>(0.5)
            : sum * (static_cast<Jitter2::Real>(1.0) / static_cast<Jitter2::Real>(count));
    }

    void QueueNearestBreakableToCamera(JVector cameraPosition, JVector cameraDirection)
    {
        cameraDirection = NormalizeSafe(cameraDirection);

        Breakable* best = nullptr;
        Jitter2::Real bestScore = std::numeric_limits<Jitter2::Real>::max();

        for (auto& pair : Breakables)
        {
            Breakable& breakable = *pair.second;
            JVector toBody = breakable.Body->Position() - cameraPosition;
            Jitter2::Real along = JVector::Dot(toBody, cameraDirection);
            if (along < static_cast<Jitter2::Real>(0.0))
            {
                continue;
            }

            JVector lateral = toBody - cameraDirection * along;
            Jitter2::Real score = lateral.LengthSquared() + along * static_cast<Jitter2::Real>(0.02);

            if (score < bestScore)
            {
                bestScore = score;
                best = &breakable;
            }
        }

        if (best == nullptr)
        {
            return;
        }

        JVector impactPoint = best->Body->Position() - cameraDirection * best->Radius;
        QueueBreak(*best, impactPoint, cameraDirection);
    }

    void QueueBreak(Breakable& breakable, JVector impactPoint, JVector impactDirection)
    {
        if (breakable.Queued)
        {
            return;
        }

        breakable.Queued = true;
        breakable.ImpactPoint = impactPoint;
        breakable.ImpactDirection = impactDirection;
        PendingBreaks.push_back(&breakable);
    }

    void OnRealtimeFracturePostStep(Jitter2::Real dt)
    {
        for (auto& pair : Breakables)
        {
            pair.second->TimeSinceCreated += dt;
        }

        if (PendingBreaks.empty())
        {
            return;
        }

        for (Breakable* breakable : PendingBreaks)
        {
            if (breakable != nullptr && breakable->Body != nullptr && Breakables.contains(breakable->Body))
            {
                Fracture(*breakable);
            }
        }

        PendingBreaks.clear();
        if (FractureRenderer != nullptr)
        {
            FractureRenderer->SetFragments(FractureFragments);
        }
    }

    void Fracture(Breakable& breakable)
    {
        Jitter2::RigidBody& source = *breakable.Body;

        JVector sourcePosition = source.Position();
        JQuaternion sourceOrientation = source.Orientation();
        JVector sourceVelocity = source.Velocity();
        JVector sourceAngularVelocity = source.AngularVelocity();
        Jitter2::Real sourceFriction = source.Friction();
        Jitter2::Real sourceRestitution = source.Restitution();
        JVector localImpact = JQuaternion::ConjugatedTransform(breakable.ImpactPoint - sourcePosition, sourceOrientation);
        JVector localDirection = JQuaternion::ConjugatedTransform(breakable.ImpactDirection, sourceOrientation);
        int sourceSeed = breakable.Seed;
        int nextGeneration = breakable.Generation + 1;
        ConvexPolyhedron original = breakable.Source;

        std::vector<JVector> sites = GenerateSites(breakable, localImpact, localDirection, sourceSeed);
        std::vector<PendingFragment> pending;

        for (int i = 0; i < static_cast<int>(sites.size()); ++i)
        {
            ConvexPolyhedron cell = original.Clone();

            for (int j = 0; j < static_cast<int>(sites.size()); ++j)
            {
                if (i == j)
                {
                    continue;
                }

                JVector normal = sites[static_cast<std::size_t>(j)] - sites[static_cast<std::size_t>(i)];
                JVector midpoint = (sites[static_cast<std::size_t>(i)] + sites[static_cast<std::size_t>(j)])
                    * static_cast<Jitter2::Real>(0.5);
                Jitter2::Real offset = JVector::Dot(normal, midpoint);

                if (!Clip(cell, normal, offset))
                {
                    cell.Faces.clear();
                    break;
                }
            }

            std::vector<Jitter2::LinearMath::JTriangle> triangles = Triangulate(cell);
            WeldTriangles(triangles);
            if (triangles.size() < 4)
            {
                continue;
            }

            try
            {
                std::vector<JVector> points = ExtractUniqueVertices(triangles);
                if (points.size() < 4)
                {
                    continue;
                }

                Shapes::PointCloudShape measureShape(points);
                Jitter2::LinearMath::JMatrix inertia;
                JVector centerOfMass;
                Jitter2::Real mass = 0;
                measureShape.CalculateMassInertia(inertia, centerOfMass, mass);
                if (mass < MinPieceMass || !IsFinite(centerOfMass))
                {
                    continue;
                }

                ConvexPolyhedron shiftedSource = ShiftPolyhedron(cell, centerOfMass);
                FractureBounds bounds = CalculateBounds(shiftedSource);
                std::vector<JVector> localVertices = ShiftTriangles(triangles, centerOfMass);
                pending.push_back(PendingFragment {
                    centerOfMass,
                    points,
                    localVertices,
                    shiftedSource,
                    bounds.Min,
                    bounds.Max,
                    bounds.Radius,
                    mass,
                });
            }
            catch (const std::exception&)
            {
            }
        }

        if (pending.empty())
        {
            breakable.Queued = false;
            return;
        }

        source.BeginCollide = {};
        Breakables.erase(&source);
        FractureFragments.erase(
            std::remove_if(
                FractureFragments.begin(),
                FractureFragments.end(),
                [&source](const FractureFragment& fragment)
                {
                    return fragment.Body == &source;
                }),
            FractureFragments.end());
        ClearGrabIfBody(source);
        World.Remove(source);

        DotNetRandom random(CombineFragmentSeed(sourceSeed));

        for (PendingFragment& fragment : pending)
        {
            int fragmentSeed = NextRandomInt(random);
            Shapes::PointCloudShape& shape = CreateShape<Shapes::PointCloudShape>(fragment.Points);
            shape.Shift(-fragment.CenterOfMass);

            JVector worldOffset = JQuaternion::Transform(fragment.CenterOfMass, sourceOrientation);
            Jitter2::RigidBody& body = World.CreateRigidBody();
            body.Position(sourcePosition + worldOffset);
            body.Orientation(sourceOrientation);
            body.Velocity(sourceVelocity + JVector::Cross(sourceAngularVelocity, worldOffset));
            body.AngularVelocity(sourceAngularVelocity + RandomVector(random, static_cast<Jitter2::Real>(4.0)));
            body.Friction(sourceFriction);
            body.Restitution(sourceRestitution);
            body.Tag = RigidBodyTag {};
            body.AddShape(shape);

            if (CanRefracture(fragment, nextGeneration))
            {
                body.BeginCollide =
                    [this](Jitter2::Arbiter& arbiter)
                    {
                        OnBreakableCollide(arbiter);
                    };

                auto child = std::make_unique<Breakable>();
                child->Body = &body;
                child->Size = fragment.BoundsMax - fragment.BoundsMin;
                child->BoundsMin = fragment.BoundsMin;
                child->BoundsMax = fragment.BoundsMax;
                child->Radius = fragment.Radius;
                child->Seed = fragmentSeed;
                child->Generation = nextGeneration;
                child->Source = fragment.Source;
                Breakables.emplace(&body, std::move(child));
            }

            JVector localBurst = NormalizeSafe(fragment.CenterOfMass - localImpact);
            if (localBurst.LengthSquared() > static_cast<Jitter2::Real>(0.5))
            {
                JVector burst = JQuaternion::Transform(localBurst, sourceOrientation);
                body.ApplyImpulse(burst * (static_cast<Jitter2::Real>(0.08)
                    + static_cast<Jitter2::Real>(0.02) * fragment.Mass));
            }

            FractureFragments.push_back(FractureFragment {
                &body,
                fragment.LocalVertices,
                ColorGenerator::GetColor(static_cast<int>(std::hash<const Jitter2::RigidBody*>{}(&body))),
                0,
            });
        }
    }

    static bool CanRefracture(const PendingFragment& fragment, int generation)
    {
        return generation <= MaxFractureGeneration
            && fragment.Mass >= MinRefractureMass
            && fragment.Radius >= MinRefractureRadius
            && fragment.Source.Faces.size() >= 4;
    }

    static std::vector<JVector> GenerateSites(
        const Breakable& breakable,
        JVector impact,
        JVector direction,
        int seed)
    {
        return breakable.SphericalSites
            ? GenerateSphereSites(breakable.Radius, impact, direction, seed)
            : GenerateBoxSites(breakable.BoundsMin, breakable.BoundsMax, impact, direction, seed);
    }

    static std::vector<JVector> GenerateBoxSites(
        JVector boundsMin,
        JVector boundsMax,
        JVector impact,
        JVector direction,
        int seed)
    {
        DotNetRandom random(seed);
        JVector center = (boundsMin + boundsMax) * static_cast<Jitter2::Real>(0.5);
        JVector half = (boundsMax - boundsMin) * static_cast<Jitter2::Real>(0.5);
        JVector size = half * static_cast<Jitter2::Real>(2.0);
        JVector innerMin = center - half * static_cast<Jitter2::Real>(0.92);
        JVector innerMax = center + half * static_cast<Jitter2::Real>(0.92);
        JVector randomMin = center - half * static_cast<Jitter2::Real>(0.94);
        JVector randomMax = center + half * static_cast<Jitter2::Real>(0.94);
        std::vector<JVector> sites;
        sites.reserve(FractureSiteCount);

        sites.push_back(ClampToBounds(impact, innerMin, innerMax));

        for (int i = 1; i < FractureSiteCount; ++i)
        {
            JVector point;

            if (i < FractureSiteCount * 2 / 3)
            {
                JVector offset = RandomVector(random, static_cast<Jitter2::Real>(0.35)
                    + static_cast<Jitter2::Real>(0.75) * NextFloat(random));
                offset += direction * ((NextFloat(random) - static_cast<Jitter2::Real>(0.5))
                    * static_cast<Jitter2::Real>(0.5));
                point = impact + JVector(offset.X * size.X, offset.Y * size.Y, offset.Z * size.Z);
            }
            else
            {
                point = JVector(
                    randomMin.X + NextFloat(random) * (randomMax.X - randomMin.X),
                    randomMin.Y + NextFloat(random) * (randomMax.Y - randomMin.Y),
                    randomMin.Z + NextFloat(random) * (randomMax.Z - randomMin.Z));
            }

            sites.push_back(ClampToBounds(point, randomMin, randomMax));
        }

        return sites;
    }

    static std::vector<JVector> GenerateSphereSites(Jitter2::Real radius, JVector impact, JVector direction, int seed)
    {
        DotNetRandom random(seed);
        std::vector<JVector> sites;
        sites.reserve(FractureSiteCount);

        sites.push_back(ClampToSphere(impact, radius * static_cast<Jitter2::Real>(0.92)));

        for (int i = 1; i < FractureSiteCount; ++i)
        {
            JVector point;

            if (i < FractureSiteCount * 2 / 3)
            {
                JVector offset = RandomVector(
                    random,
                    radius * (static_cast<Jitter2::Real>(0.25) + static_cast<Jitter2::Real>(0.65) * NextFloat(random)));
                offset += direction * ((NextFloat(random) - static_cast<Jitter2::Real>(0.5))
                    * radius * static_cast<Jitter2::Real>(0.45));
                point = impact + offset;
            }
            else
            {
                point = RandomPointInSphere(random, radius * static_cast<Jitter2::Real>(0.9));
            }

            sites.push_back(ClampToSphere(point, radius * static_cast<Jitter2::Real>(0.94)));
        }

        return sites;
    }

    static ConvexPolyhedron CreateBoxPolyhedron(JVector size)
    {
        JVector h = size * static_cast<Jitter2::Real>(0.5);

        JVector nnn(-h.X, -h.Y, -h.Z);
        JVector nnp(-h.X, -h.Y, +h.Z);
        JVector npn(-h.X, +h.Y, -h.Z);
        JVector npp(-h.X, +h.Y, +h.Z);
        JVector pnn(+h.X, -h.Y, -h.Z);
        JVector pnp(+h.X, -h.Y, +h.Z);
        JVector ppn(+h.X, +h.Y, -h.Z);
        JVector ppp(+h.X, +h.Y, +h.Z);

        ConvexPolyhedron poly;
        poly.Faces.emplace_back(std::vector<JVector> {pnn, ppn, ppp, pnp});
        poly.Faces.emplace_back(std::vector<JVector> {nnn, nnp, npp, npn});
        poly.Faces.emplace_back(std::vector<JVector> {npn, npp, ppp, ppn});
        poly.Faces.emplace_back(std::vector<JVector> {nnn, pnn, pnp, nnp});
        poly.Faces.emplace_back(std::vector<JVector> {nnp, pnp, ppp, npp});
        poly.Faces.emplace_back(std::vector<JVector> {nnn, npn, ppn, pnn});
        return poly;
    }

    static ConvexPolyhedron CreateSpherePolyhedron(Jitter2::Real radius)
    {
        constexpr int latitudeBands = 8;
        constexpr int longitudeBands = 16;

        JVector top(0, radius, 0);
        JVector bottom(0, -radius, 0);
        std::array<std::array<JVector, longitudeBands>, latitudeBands - 1> rings {};

        for (int latitude = 1; latitude < latitudeBands; ++latitude)
        {
            Jitter2::Real theta = static_cast<Jitter2::Real>(Pi) * static_cast<Jitter2::Real>(latitude)
                / static_cast<Jitter2::Real>(latitudeBands);
            Jitter2::Real y = std::cos(theta) * radius;
            Jitter2::Real ringRadius = std::sin(theta) * radius;

            for (int longitude = 0; longitude < longitudeBands; ++longitude)
            {
                Jitter2::Real phi = static_cast<Jitter2::Real>(2.0f * Pi)
                    * static_cast<Jitter2::Real>(longitude) / static_cast<Jitter2::Real>(longitudeBands);
                rings[static_cast<std::size_t>(latitude - 1)][static_cast<std::size_t>(longitude)] =
                    JVector(std::cos(phi) * ringRadius, y, std::sin(phi) * ringRadius);
            }
        }

        ConvexPolyhedron poly;

        for (int longitude = 0; longitude < longitudeBands; ++longitude)
        {
            int next = (longitude + 1) % longitudeBands;
            poly.Faces.emplace_back(std::vector<JVector> {
                top,
                rings[0][static_cast<std::size_t>(next)],
                rings[0][static_cast<std::size_t>(longitude)],
            });
        }

        for (int latitude = 0; latitude < latitudeBands - 2; ++latitude)
        {
            for (int longitude = 0; longitude < longitudeBands; ++longitude)
            {
                int next = (longitude + 1) % longitudeBands;

                JVector a = rings[static_cast<std::size_t>(latitude)][static_cast<std::size_t>(longitude)];
                JVector b = rings[static_cast<std::size_t>(latitude)][static_cast<std::size_t>(next)];
                JVector c = rings[static_cast<std::size_t>(latitude + 1)][static_cast<std::size_t>(next)];
                JVector d = rings[static_cast<std::size_t>(latitude + 1)][static_cast<std::size_t>(longitude)];

                poly.Faces.emplace_back(std::vector<JVector> {a, b, c});
                poly.Faces.emplace_back(std::vector<JVector> {a, c, d});
            }
        }

        constexpr int last = latitudeBands - 2;
        for (int longitude = 0; longitude < longitudeBands; ++longitude)
        {
            int next = (longitude + 1) % longitudeBands;
            poly.Faces.emplace_back(std::vector<JVector> {
                bottom,
                rings[last][static_cast<std::size_t>(longitude)],
                rings[last][static_cast<std::size_t>(next)],
            });
        }

        return poly;
    }

    static bool Clip(ConvexPolyhedron& poly, JVector normal, Jitter2::Real offset)
    {
        std::vector<JVector> capVertices;

        for (int i = static_cast<int>(poly.Faces.size()); i-- > 0;)
        {
            FractureFace& face = poly.Faces[static_cast<std::size_t>(i)];
            std::vector<JVector> clipped = ClipFace(face.Vertices, normal, offset, capVertices);

            if (clipped.size() < 3)
            {
                poly.Faces.erase(poly.Faces.begin() + i);
            }
            else
            {
                face.Vertices = std::move(clipped);
            }
        }

        RemoveDuplicatePoints(capVertices);

        if (capVertices.size() >= 3)
        {
            FractureFace cap = CreateCapFace(capVertices, normal);
            if (cap.Vertices.size() >= 3)
            {
                poly.Faces.push_back(std::move(cap));
            }
        }

        return poly.Faces.size() >= 4;
    }

    static std::vector<JVector> ClipFace(
        const std::vector<JVector>& vertices,
        JVector normal,
        Jitter2::Real offset,
        std::vector<JVector>& capVertices)
    {
        std::vector<JVector> result;
        result.reserve(vertices.size() + 1);
        if (vertices.empty())
        {
            return result;
        }

        JVector previous = vertices.back();
        Jitter2::Real previousDistance = JVector::Dot(normal, previous) - offset;
        bool previousInside = previousDistance <= PlaneEpsilon;

        for (const JVector& current : vertices)
        {
            Jitter2::Real currentDistance = JVector::Dot(normal, current) - offset;
            bool currentInside = currentDistance <= PlaneEpsilon;

            if (currentInside != previousInside)
            {
                Jitter2::Real t = previousDistance / (previousDistance - currentDistance);
                JVector intersection = previous + (current - previous) * t;
                AddUnique(result, intersection);
                AddUnique(capVertices, intersection);
            }

            if (currentInside)
            {
                AddUnique(result, current);
            }

            previous = current;
            previousDistance = currentDistance;
            previousInside = currentInside;
        }

        CleanupPolygon(result);
        return result;
    }

    static FractureFace CreateCapFace(std::vector<JVector>& vertices, JVector normal)
    {
        JVector n = NormalizeSafe(normal);
        if (n.LengthSquared() < static_cast<Jitter2::Real>(0.5))
        {
            return FractureFace {};
        }

        JVector center = JVector::Zero();
        for (const JVector& vertex : vertices)
        {
            center += vertex;
        }
        center *= static_cast<Jitter2::Real>(1.0) / static_cast<Jitter2::Real>(vertices.size());

        JVector axis = std::abs(n.Y) < static_cast<Jitter2::Real>(0.8) ? JVector::UnitY() : JVector::UnitX();
        JVector u = NormalizeSafe(JVector::Cross(axis, n));
        JVector v = JVector::Cross(n, u);

        std::sort(
            vertices.begin(),
            vertices.end(),
            [center, u, v](const JVector& a, const JVector& b)
            {
                JVector da = a - center;
                JVector db = b - center;
                Jitter2::Real aa = std::atan2(JVector::Dot(da, v), JVector::Dot(da, u));
                Jitter2::Real bb = std::atan2(JVector::Dot(db, v), JVector::Dot(db, u));
                return aa < bb;
            });

        CleanupPolygon(vertices);
        return FractureFace(vertices);
    }

    static std::vector<Jitter2::LinearMath::JTriangle> Triangulate(const ConvexPolyhedron& poly)
    {
        std::vector<Jitter2::LinearMath::JTriangle> triangles;
        JVector inside = JVector::Zero();
        int vertexCount = 0;

        for (const FractureFace& face : poly.Faces)
        {
            for (const JVector& vertex : face.Vertices)
            {
                inside += vertex;
                ++vertexCount;
            }
        }

        if (vertexCount == 0)
        {
            return triangles;
        }
        inside *= static_cast<Jitter2::Real>(1.0) / static_cast<Jitter2::Real>(vertexCount);

        for (const FractureFace& face : poly.Faces)
        {
            const std::vector<JVector>& vertices = face.Vertices;
            if (vertices.size() < 3)
            {
                continue;
            }

            for (int j = 1; j < static_cast<int>(vertices.size()) - 1; ++j)
            {
                JVector a = vertices[0];
                JVector b = vertices[static_cast<std::size_t>(j)];
                JVector c = vertices[static_cast<std::size_t>(j + 1)];

                JVector normal = JVector::Cross(b - a, c - a);
                if (normal.LengthSquared() < static_cast<Jitter2::Real>(1e-10))
                {
                    continue;
                }

                if (JVector::Dot(normal, a - inside) < static_cast<Jitter2::Real>(0.0))
                {
                    std::swap(b, c);
                }

                triangles.emplace_back(a, b, c);
            }
        }

        return triangles;
    }

    static void WeldTriangles(std::vector<Jitter2::LinearMath::JTriangle>& triangles)
    {
        std::vector<JVector> vertices;
        std::vector<Jitter2::LinearMath::JTriangle> welded;
        welded.reserve(triangles.size());

        for (const Jitter2::LinearMath::JTriangle& triangle : triangles)
        {
            JVector a = GetCanonicalVertex(vertices, triangle.V0);
            JVector b = GetCanonicalVertex(vertices, triangle.V1);
            JVector c = GetCanonicalVertex(vertices, triangle.V2);

            if ((a - b).LengthSquared() < MergeEpsilonSquared
                || (b - c).LengthSquared() < MergeEpsilonSquared
                || (c - a).LengthSquared() < MergeEpsilonSquared)
            {
                continue;
            }

            JVector normal = JVector::Cross(b - a, c - a);
            if (normal.LengthSquared() < static_cast<Jitter2::Real>(1e-10))
            {
                continue;
            }

            welded.emplace_back(a, b, c);
        }

        triangles = std::move(welded);
    }

    static std::vector<JVector> ExtractUniqueVertices(const std::vector<Jitter2::LinearMath::JTriangle>& triangles)
    {
        std::vector<JVector> vertices;

        for (const Jitter2::LinearMath::JTriangle& triangle : triangles)
        {
            GetCanonicalVertex(vertices, triangle.V0);
            GetCanonicalVertex(vertices, triangle.V1);
            GetCanonicalVertex(vertices, triangle.V2);
        }

        return vertices;
    }

    static JVector GetCanonicalVertex(std::vector<JVector>& vertices, JVector vertex)
    {
        for (const JVector& existing : vertices)
        {
            if ((existing - vertex).LengthSquared() < MergeEpsilonSquared)
            {
                return existing;
            }
        }

        vertices.push_back(vertex);
        return vertex;
    }

    static std::vector<JVector> ShiftTriangles(
        const std::vector<Jitter2::LinearMath::JTriangle>& triangles,
        JVector centerOfMass)
    {
        std::vector<JVector> localVertices;
        localVertices.reserve(triangles.size() * 3);

        for (const Jitter2::LinearMath::JTriangle& triangle : triangles)
        {
            localVertices.push_back(triangle.V0 - centerOfMass);
            localVertices.push_back(triangle.V1 - centerOfMass);
            localVertices.push_back(triangle.V2 - centerOfMass);
        }

        return localVertices;
    }

    static ConvexPolyhedron ShiftPolyhedron(const ConvexPolyhedron& poly, JVector offset)
    {
        ConvexPolyhedron shifted;

        for (const FractureFace& face : poly.Faces)
        {
            std::vector<JVector> shiftedVertices;
            shiftedVertices.reserve(face.Vertices.size());

            for (const JVector& vertex : face.Vertices)
            {
                shiftedVertices.push_back(vertex - offset);
            }

            shifted.Faces.emplace_back(std::move(shiftedVertices));
        }

        return shifted;
    }

    static FractureBounds CalculateBounds(const ConvexPolyhedron& poly)
    {
        JVector min = JVector::MaxValue();
        JVector max = JVector::MinValue();
        Jitter2::Real radiusSquared = static_cast<Jitter2::Real>(0.0);
        bool hasVertex = false;

        for (const FractureFace& face : poly.Faces)
        {
            for (const JVector& vertex : face.Vertices)
            {
                if (!IsFinite(vertex))
                {
                    continue;
                }

                min = JVector::Min(min, vertex);
                max = JVector::Max(max, vertex);

                radiusSquared = std::max(radiusSquared, vertex.LengthSquared());
                hasVertex = true;
            }
        }

        return hasVertex
            ? FractureBounds {min, max, std::sqrt(radiusSquared)}
            : FractureBounds {JVector::Zero(), JVector::Zero(), static_cast<Jitter2::Real>(0.0)};
    }

    static void CleanupPolygon(std::vector<JVector>& vertices)
    {
        for (int i = static_cast<int>(vertices.size()); i-- > 0 && vertices.size() > 1;)
        {
            JVector a = vertices[static_cast<std::size_t>(i)];
            JVector b = vertices[static_cast<std::size_t>((i + 1) % static_cast<int>(vertices.size()))];
            if ((a - b).LengthSquared() < MergeEpsilonSquared)
            {
                vertices.erase(vertices.begin() + i);
            }
        }
    }

    static void RemoveDuplicatePoints(std::vector<JVector>& vertices)
    {
        for (int i = static_cast<int>(vertices.size()); i-- > 0;)
        {
            for (int j = 0; j < i; ++j)
            {
                if ((vertices[static_cast<std::size_t>(i)] - vertices[static_cast<std::size_t>(j)]).LengthSquared()
                    < MergeEpsilonSquared)
                {
                    vertices.erase(vertices.begin() + i);
                    break;
                }
            }
        }
    }

    static void AddUnique(std::vector<JVector>& vertices, JVector point)
    {
        for (const JVector& vertex : vertices)
        {
            if ((vertex - point).LengthSquared() < MergeEpsilonSquared)
            {
                return;
            }
        }

        vertices.push_back(point);
    }

    static JVector ClampToBounds(JVector point, JVector min, JVector max)
    {
        return JVector(
            std::clamp(point.X, min.X, max.X),
            std::clamp(point.Y, min.Y, max.Y),
            std::clamp(point.Z, min.Z, max.Z));
    }

    static JVector ClampToBox(JVector point, JVector halfSize)
    {
        return JVector(
            std::clamp(point.X, -halfSize.X, +halfSize.X),
            std::clamp(point.Y, -halfSize.Y, +halfSize.Y),
            std::clamp(point.Z, -halfSize.Z, +halfSize.Z));
    }

    static JVector ClampToSphere(JVector point, Jitter2::Real radius)
    {
        if (radius <= static_cast<Jitter2::Real>(0.0))
        {
            return JVector::Zero();
        }

        Jitter2::Real lengthSquared = point.LengthSquared();
        Jitter2::Real radiusSquared = radius * radius;
        if (lengthSquared <= radiusSquared || lengthSquared < static_cast<Jitter2::Real>(1e-10))
        {
            return point;
        }

        return point * (radius / std::sqrt(lengthSquared));
    }

    class DotNetRandom
    {
    public:
        explicit DotNetRandom(int seed)
        {
            int subtraction = seed == std::numeric_limits<int>::min()
                ? std::numeric_limits<int>::max()
                : std::abs(seed);
            int mj = MSeed - subtraction;
            if (mj < 0)
            {
                mj += MBig;
            }

            seedArray_[55] = mj;
            int mk = 1;

            for (int i = 1; i < 55; ++i)
            {
                const int ii = (21 * i) % 55;
                seedArray_[ii] = mk;
                mk = mj - mk;
                if (mk < 0)
                {
                    mk += MBig;
                }
                mj = seedArray_[ii];
            }

            for (int k = 1; k < 5; ++k)
            {
                for (int i = 1; i < 56; ++i)
                {
                    seedArray_[i] -= seedArray_[1 + (i + 30) % 55];
                    if (seedArray_[i] < 0)
                    {
                        seedArray_[i] += MBig;
                    }
                }
            }
        }

        int Next()
        {
            return InternalSample();
        }

        double NextDouble()
        {
            return static_cast<double>(InternalSample()) * (1.0 / static_cast<double>(MBig));
        }

    private:
        int InternalSample()
        {
            int locINext = inext_;
            if (++locINext >= 56)
            {
                locINext = 1;
            }

            int locINextp = inextp_;
            if (++locINextp >= 56)
            {
                locINextp = 1;
            }

            int result = seedArray_[static_cast<std::size_t>(locINext)]
                - seedArray_[static_cast<std::size_t>(locINextp)];
            if (result == MBig)
            {
                --result;
            }
            if (result < 0)
            {
                result += MBig;
            }

            seedArray_[static_cast<std::size_t>(locINext)] = result;
            inext_ = locINext;
            inextp_ = locINextp;
            return result;
        }

        static constexpr int MBig = 2147483647;
        static constexpr int MSeed = 161803398;

        std::array<int, 56> seedArray_ {};
        int inext_ = 0;
        int inextp_ = 21;
    };

    static int ToInt32(std::uint32_t value)
    {
        if (value <= 0x7fffffffu)
        {
            return static_cast<int>(value);
        }

        if (value == 0x80000000u)
        {
            return std::numeric_limits<int>::min();
        }

        return -static_cast<int>((~value + 1u) & 0x7fffffffu);
    }

    static int CombineFragmentSeed(int sourceSeed)
    {
        return ToInt32(static_cast<std::uint32_t>(sourceSeed) * 17u + 5u);
    }

    static JVector RandomPointInSphere(DotNetRandom& random, Jitter2::Real radius)
    {
        for (int i = 0; i < 16; ++i)
        {
            JVector point(
                (NextFloat(random) * static_cast<Jitter2::Real>(2.0) - static_cast<Jitter2::Real>(1.0)) * radius,
                (NextFloat(random) * static_cast<Jitter2::Real>(2.0) - static_cast<Jitter2::Real>(1.0)) * radius,
                (NextFloat(random) * static_cast<Jitter2::Real>(2.0) - static_cast<Jitter2::Real>(1.0)) * radius);

            if (point.LengthSquared() <= radius * radius)
            {
                return point;
            }
        }

        return JVector::Zero();
    }

    static JVector RandomVector(DotNetRandom& random, Jitter2::Real scale)
    {
        for (int i = 0; i < 8; ++i)
        {
            JVector v(
                NextFloat(random) * static_cast<Jitter2::Real>(2.0) - static_cast<Jitter2::Real>(1.0),
                NextFloat(random) * static_cast<Jitter2::Real>(2.0) - static_cast<Jitter2::Real>(1.0),
                NextFloat(random) * static_cast<Jitter2::Real>(2.0) - static_cast<Jitter2::Real>(1.0));

            if (v.LengthSquared() > static_cast<Jitter2::Real>(1e-6))
            {
                return JVector::Normalize(v) * (scale * NextFloat(random));
            }
        }

        return JVector::Zero();
    }

    static bool IsFinite(JVector value)
    {
        return std::isfinite(value.X) && std::isfinite(value.Y) && std::isfinite(value.Z);
    }

    static JVector NormalizeSafe(JVector value)
    {
        const Jitter2::Real lengthSquared = value.LengthSquared();
        if (lengthSquared <= static_cast<Jitter2::Real>(1e-12))
        {
            return JVector::Zero();
        }

        return value / std::sqrt(lengthSquared);
    }

    static Jitter2::Real NextFloat(DotNetRandom& random)
    {
        return static_cast<Jitter2::Real>(static_cast<float>(random.NextDouble()));
    }

    static int NextRandomInt(DotNetRandom& random)
    {
        return random.Next();
    }

    static constexpr int FractureSiteCount = 16;
    static constexpr int MaxFractureGeneration = 2;
    static constexpr Jitter2::Real BreakSpeed = static_cast<Jitter2::Real>(8.0);
    static constexpr Jitter2::Real MinPieceMass = static_cast<Jitter2::Real>(0.015);
    static constexpr Jitter2::Real MinRefractureMass = static_cast<Jitter2::Real>(0.08);
    static constexpr Jitter2::Real MinRefractureRadius = static_cast<Jitter2::Real>(0.38);
    static constexpr Jitter2::Real RefractureArmDelay = static_cast<Jitter2::Real>(0.04);
    static constexpr Jitter2::Real PlaneEpsilon = static_cast<Jitter2::Real>(1e-5);
    static constexpr Jitter2::Real MergeEpsilonSquared = static_cast<Jitter2::Real>(1e-8);
