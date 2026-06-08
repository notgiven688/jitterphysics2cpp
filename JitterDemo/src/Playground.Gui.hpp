void UpdateCameraDirection(CameraState& camera)
{
    camera.Theta = std::clamp(
        camera.Theta,
        static_cast<double>(0.1),
        static_cast<double>(Pi) - static_cast<double>(0.1));

    camera.Direction = Vec3 {
        -static_cast<float>(std::sin(camera.Theta) * std::sin(camera.Phi)),
        static_cast<float>(std::cos(camera.Theta)),
        -static_cast<float>(std::sin(camera.Theta) * std::cos(camera.Phi)),
    };
}

void UpdateCameraInput(GLFWwindow* window, CameraState& camera, const ImGuiIO& io)
{
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS && !io.WantCaptureMouse)
    {
        double mouseX = 0.0;
        double mouseY = 0.0;
        glfwGetCursorPos(window, &mouseX, &mouseY);
        if (!camera.Dragging)
        {
            camera.Dragging = true;
            camera.LastMouseX = mouseX;
            camera.LastMouseY = mouseY;
        }

        const double dx = mouseX - camera.LastMouseX;
        const double dy = mouseY - camera.LastMouseY;
        camera.LastMouseX = mouseX;
        camera.LastMouseY = mouseY;

        camera.Phi -= dx * static_cast<double>(camera.MouseSensitivity);
        camera.Theta += dy * static_cast<double>(camera.MouseSensitivity);
    }
    else
    {
        camera.Dragging = false;
    }

    UpdateCameraDirection(camera);

    const Vec3 right = Normalize(Cross(Vec3 {0.0f, 1.0f, 0.0f}, camera.Direction));
    Vec3 movement {};
    if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) != GLFW_PRESS)
    {
        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) movement = movement + camera.Direction;
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS) movement = movement - camera.Direction;
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS) movement = movement + right;
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS) movement = movement - right;
    }

    if (Dot(movement, movement) > 0.1f)
    {
        movement = Normalize(movement);
    }
    camera.Position = camera.Position + movement * camera.MoveSpeed;
}

const char* GlobalControls =
    "[Controls]\n"
    "WASD - Move camera\n"
    "Right Mouse (hold) - Rotate camera\n"
    "Left Mouse (hold) - Grab object\n"
    "Scroll Wheel - Adjust grab distance\n"
    "Space - Shoot cube\n"
    "M - Toggle multi-threading";

bool HasText(const char* text)
{
    return text != nullptr && text[0] != '\0';
}

std::string BuildDemoMenuLabel(const std::vector<std::unique_ptr<IDemo>>& demos, int selectedDemoIndex)
{
    if (selectedDemoIndex < 0 || selectedDemoIndex >= static_cast<int>(demos.size()))
    {
        return "Select Demo Scene";
    }

    constexpr std::size_t maxNameLength = 18;
    std::string name = demos[static_cast<std::size_t>(selectedDemoIndex)]->Name();
    if (name.size() > maxNameLength)
    {
        name = name.substr(0, maxNameLength - 3) + "...";
    }

    char label[128] {};
    std::snprintf(label, sizeof(label), "Demo %02d - %s", selectedDemoIndex, name.c_str());
    return label;
}

ImVec4 Rgba(int r, int g, int b, int a = 255)
{
    return ImVec4(
        static_cast<float>(r) / 255.0f,
        static_cast<float>(g) / 255.0f,
        static_cast<float>(b) / 255.0f,
        static_cast<float>(a) / 255.0f);
}

void ConfigureGuiTheme()
{
    ImGuiStyle& style = ImGui::GetStyle();
    style.WindowPadding = ImVec2(8.0f, 8.0f);
    style.FramePadding = ImVec2(6.0f, 3.0f);
    style.ItemSpacing = ImVec2(4.0f, 4.0f);
    style.ItemInnerSpacing = ImVec2(4.0f, 4.0f);
    style.WindowBorderSize = 1.0f;
    style.FrameBorderSize = 1.0f;
    style.PopupBorderSize = 1.0f;
    style.WindowRounding = 0.0f;
    style.FrameRounding = 2.0f;
    style.PopupRounding = 2.0f;
    style.ScrollbarRounding = 2.0f;
    style.GrabRounding = 2.0f;
    style.WindowTitleAlign = ImVec2(0.0f, 0.5f);

    ImVec4* colors = style.Colors;
    colors[ImGuiCol_Text] = Rgba(255, 255, 255);
    colors[ImGuiCol_TextDisabled] = Rgba(228, 228, 228);
    colors[ImGuiCol_WindowBg] = Rgba(20, 20, 20, 112);
    colors[ImGuiCol_PopupBg] = Rgba(20, 20, 20, 232);
    colors[ImGuiCol_Border] = Rgba(156, 156, 156, 72);
    colors[ImGuiCol_FrameBg] = Rgba(40, 48, 60, 188);
    colors[ImGuiCol_FrameBgHovered] = Rgba(48, 58, 72, 204);
    colors[ImGuiCol_FrameBgActive] = Rgba(34, 42, 54, 216);
    colors[ImGuiCol_TitleBg] = Rgba(20, 20, 20, 80);
    colors[ImGuiCol_TitleBgActive] = Rgba(20, 20, 20, 96);
    colors[ImGuiCol_TitleBgCollapsed] = Rgba(20, 20, 20, 64);
    colors[ImGuiCol_Button] = Rgba(68, 68, 68, 120);
    colors[ImGuiCol_ButtonHovered] = Rgba(92, 92, 92, 152);
    colors[ImGuiCol_ButtonActive] = Rgba(60, 60, 60, 136);
    colors[ImGuiCol_Header] = Rgba(20, 20, 20, 22);
    colors[ImGuiCol_HeaderHovered] = Rgba(72, 118, 188, 64);
    colors[ImGuiCol_HeaderActive] = Rgba(72, 118, 188, 96);
    colors[ImGuiCol_CheckMark] = Rgba(120, 166, 232, 240);
    colors[ImGuiCol_SliderGrab] = Rgba(77, 121, 186, 222);
    colors[ImGuiCol_SliderGrabActive] = Rgba(96, 145, 216, 234);
    colors[ImGuiCol_Separator] = Rgba(196, 196, 196, 112);
    colors[ImGuiCol_ResizeGrip] = Rgba(148, 148, 148, 72);
    colors[ImGuiCol_ResizeGripHovered] = Rgba(188, 188, 188, 96);
    colors[ImGuiCol_ResizeGripActive] = Rgba(140, 140, 140, 84);
    colors[ImGuiCol_ScrollbarBg] = Rgba(80, 80, 80, 150);
    colors[ImGuiCol_ScrollbarGrab] = Rgba(40, 40, 40, 150);
    colors[ImGuiCol_ScrollbarGrabHovered] = Rgba(120, 166, 232, 150);
    colors[ImGuiCol_ScrollbarGrabActive] = Rgba(120, 166, 232, 208);
    colors[ImGuiCol_PlotHistogram] = Rgba(255, 186, 52, 240);
    colors[ImGuiCol_PlotHistogramHovered] = Rgba(255, 200, 90, 255);
}

std::string FormatText(const char* format, ...)
{
    char buffer[192] {};
    va_list args;
    va_start(args, format);
    std::vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    return buffer;
}

void TableRow(const char* label, const std::string& value)
{
    ImGui::TextUnformatted(label);
    ImGui::SameLine();

    const float rightEdge = ImGui::GetWindowContentRegionMax().x;
    const float valueWidth = ImGui::CalcTextSize(value.c_str()).x;
    const float valueX = std::max(ImGui::GetCursorPosX(), rightEdge - valueWidth);
    ImGui::SetCursorPosX(valueX);

    ImGui::PushStyleColor(ImGuiCol_Text, Rgba(228, 228, 228));
    ImGui::TextUnformatted(value.c_str());
    ImGui::PopStyleColor();
}

struct DemoGuiStats
{
    static constexpr std::size_t TimingCount = static_cast<std::size_t>(Jitter2::World::Timings::Last);

    std::array<double, TimingCount> DebugTimes {};
    std::array<float, 100> PhysicsTime {};
    double TotalTime = 0.0;
    double LastTime = 0.0;
    int SamplingRate = 5;
    int AccSteps = 0;
    std::uint16_t FrameCount = 0;
    std::uint16_t Fps = 100;
    bool DemoMenuHintDismissed = false;
};

void UpdateGuiStats(const Jitter2::World& world, DemoGuiStats& stats)
{
    const double time = ImGui::GetTime();
    if (time - stats.LastTime > 1.0)
    {
        stats.LastTime = time;
        stats.Fps = stats.FrameCount;
        stats.FrameCount = 0;
    }

    ++stats.FrameCount;
    ++stats.AccSteps;
    if (stats.AccSteps < stats.SamplingRate)
    {
        return;
    }

    stats.AccSteps = 0;
    stats.TotalTime = 0.0;
    const std::span<const double> timings = world.DebugTimings();
    for (std::size_t index = 0; index < stats.DebugTimes.size(); ++index)
    {
        const double value = index < timings.size() ? timings[index] : 0.0;
        stats.DebugTimes[index] = value;
        stats.TotalTime += value;
    }

    for (std::size_t index = stats.PhysicsTime.size(); index-- > 1;)
    {
        stats.PhysicsTime[index] = stats.PhysicsTime[index - 1];
    }
    stats.PhysicsTime[0] = static_cast<float>(stats.TotalTime);
}

float MaxPhysicsTime(const DemoGuiStats& stats)
{
    float maxTime = 0.0f;
    for (float value : stats.PhysicsTime)
    {
        maxTime = std::max(maxTime, value);
    }
    return maxTime;
}

bool BeginSection(const char* label, bool defaultOpen)
{
    ImGuiTreeNodeFlags flags = ImGuiTreeNodeFlags_SpanAvailWidth;
    if (defaultOpen)
    {
        flags |= ImGuiTreeNodeFlags_DefaultOpen;
    }
    return ImGui::CollapsingHeader(label, flags);
}

void DrawDemoTooltip(const IDemo& demo)
{
    const bool hasDescription = HasText(demo.Description());
    const bool hasControls = HasText(demo.Controls());
    if (!hasDescription && !hasControls)
    {
        return;
    }

    ImGui::BeginTooltip();
    ImGui::PushTextWrapPos(ImGui::GetFontSize() * 28.0f);
    if (hasDescription)
    {
        ImGui::TextUnformatted(demo.Description());
        ImGui::Spacing();
        ImGui::Spacing();
        ImGui::TextUnformatted(GlobalControls);
    }
    else
    {
        ImGui::TextUnformatted(GlobalControls);
    }

    if (hasControls)
    {
        ImGui::TextUnformatted(demo.Controls());
    }
    ImGui::PopTextWrapPos();
    ImGui::EndTooltip();
}

void DrawUi(
    DemoScene& scene,
    DemoSettings& settings)
{
    static DemoGuiStats stats;
    UpdateGuiStats(scene.World, stats);

    ImGui::SetNextWindowPos(ImVec2(16.0f, 16.0f), ImGuiCond_Appearing);
    ImGui::SetNextWindowSize(ImVec2(260.0f, 760.0f), ImGuiCond_Appearing);
    ImGui::SetNextWindowSizeConstraints(ImVec2(240.0f, 260.0f), ImVec2(340.0f, 900.0f));
    ImGui::SetNextWindowCollapsed(false, ImGuiCond_Always);
    ImGui::Begin(
        "Jitter2 Demo",
        nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse);

    ImGui::PushStyleColor(ImGuiCol_Text, Rgba(255, 200, 90));
    ImGui::Text("%hu fps", stats.Fps);
    ImGui::PopStyleColor();

    const auto& demos = scene.Demos();
    const std::string demoMenuLabel = BuildDemoMenuLabel(demos, settings.DemoIndex);
    if (!stats.DemoMenuHintDismissed)
    {
        const float pulse = 0.5f + 0.5f * std::sin(static_cast<float>(ImGui::GetTime()) * 4.0f);
        const float base = 110.0f / 255.0f;
        const float value = base + (1.0f - base) * pulse;
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(value, value, value, 1.0f));
    }

    constexpr const char* popupId = "demo-menu-popup";
    const float menuWidth = ImGui::GetContentRegionAvail().x;
    const bool menuPressed = ImGui::Button(demoMenuLabel.c_str(), ImVec2(menuWidth, 0.0f));
    const bool menuHovered = ImGui::IsItemHovered();
    const ImVec2 menuMin = ImGui::GetItemRectMin();
    const ImVec2 menuMax = ImGui::GetItemRectMax();
    if (!stats.DemoMenuHintDismissed)
    {
        ImGui::PopStyleColor();
    }

    if (menuPressed || (menuHovered && !ImGui::IsPopupOpen(popupId)))
    {
        ImGui::SetNextWindowPos(ImVec2(menuMax.x + 4.0f, menuMin.y), ImGuiCond_Appearing);
        ImGui::OpenPopup(popupId);
    }

    ImGui::SetNextWindowSizeConstraints(ImVec2(menuWidth, 0.0f), ImVec2(FLT_MAX, 720.0f));
    const bool menuOpen = ImGui::BeginPopup(popupId);
    if (menuOpen)
    {
        for (int index = 0; index < static_cast<int>(demos.size()); ++index)
        {
            const IDemo& demo = *demos[static_cast<std::size_t>(index)];
            char itemLabel[160] {};
            std::snprintf(itemLabel, sizeof(itemLabel), "Demo %02d - %s", index, demo.Name());
            const bool selected = settings.DemoIndex == index;
            if (ImGui::MenuItem(itemLabel, nullptr, selected))
            {
                settings.DemoIndex = index;
                scene.SwitchDemo(settings.DemoIndex);
            }
            if (ImGui::IsItemHovered())
            {
                DrawDemoTooltip(demo);
            }
            if (selected)
            {
                ImGui::SetItemDefaultFocus();
            }
        }
        ImGui::EndPopup();
    }

    if (!stats.DemoMenuHintDismissed && (menuHovered || menuOpen))
    {
        stats.DemoMenuHintDismissed = true;
    }

    ImGui::Separator();

    if (BeginSection("Objects", true))
    {
        TableRow("Islands", FormatText("%zu/%zu", scene.World.Islands().Count(), scene.World.Islands().ActiveCount()));
        TableRow("Bodies", FormatText("%zu/%zu", scene.World.RigidBodyDataCount(), scene.World.ActiveRigidBodyCount()));
        TableRow("Arbiter", FormatText("%zu/%zu", scene.World.ContactCount(), scene.World.ActiveContactCount()));
        TableRow("Constraints", FormatText("%zu/%zu", scene.World.ConstraintDataCount(), scene.World.ActiveConstraintDataCount()));
        TableRow("SmallConstraints", FormatText("%zu/%zu", scene.World.SmallConstraintDataCount(), scene.World.ActiveSmallConstraintDataCount()));
        TableRow("Proxies", FormatText("%zu/%zu", scene.World.DynamicTree().Count(), scene.World.DynamicTree().ActiveCount()));
    }

    if (BeginSection("Options", true))
    {
        ImGui::Checkbox("Allow Deactivation", &scene.World.AllowDeactivation);
        ImGui::Checkbox("Auxiliary Flat Surface", &scene.World.EnableAuxiliaryContactPoints);
        ImGui::Checkbox("Multithreading", &settings.Multithread);
    }

    if (BeginSection("Debug Draw", false))
    {
        ImGui::Checkbox("Islands", &settings.DebugDrawIslands);
        ImGui::Checkbox("Contacts", &settings.DebugDrawContacts);
        ImGui::Checkbox("Shapes", &settings.DebugDrawShapes);
    }

    if (BeginSection("Broadphase", false))
    {
        const auto [slotCount, pairCount] = scene.World.DynamicTree().HashSetInfo();
        TableRow("PairHashSet Size", FormatText("%zu", slotCount));
        TableRow("PairHashSet Count", FormatText("%zu", pairCount));
        TableRow("Proxies Updated", FormatText("%zu", scene.World.DynamicTree().UpdatedProxyCount()));

        const std::span<const double> timings = scene.World.DynamicTree().DebugTimings();
        for (std::size_t index = 0; index < timings.size(); ++index)
        {
            const auto timing = static_cast<Jitter2::Collision::DynamicTree::Timings>(index);
            TableRow(
                Jitter2::Collision::DynamicTree::TimingName(timing),
                FormatText("%.2f", timings[index]));
        }

        ImGui::Spacing();
        ImGui::Checkbox("Debug draw tree", &settings.DebugDrawTree);
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        if (ImGui::SliderInt("##tree-depth", &settings.DebugDrawTreeDepth, 1, 64, "tree depth: %d"))
        {
            settings.DebugDrawTree = true;
        }
    }

    if (BeginSection("Timings", true))
    {
        for (std::size_t index = 0; index < stats.DebugTimes.size(); ++index)
        {
            const auto timing = static_cast<Jitter2::World::Timings>(index);
            TableRow(Jitter2::World::TimingName(timing), FormatText("%.2f", stats.DebugTimes[index]));
        }

        ImGui::Spacing();
        const float maxPhysicsTime = MaxPhysicsTime(stats);
        const std::string overlay = FormatText("max. %.2f ms", static_cast<double>(maxPhysicsTime));
        ImGui::PlotHistogram(
            "##physics-time",
            stats.PhysicsTime.data(),
            static_cast<int>(stats.PhysicsTime.size()),
            0,
            overlay.c_str(),
            0.0f,
            maxPhysicsTime > 0.0f ? maxPhysicsTime : 1.0f,
            ImVec2(ImGui::GetContentRegionAvail().x, 80.0f));
        TableRow(
            "Total",
            FormatText(
                "%.2fms (%.0f fps)",
                stats.TotalTime,
                stats.TotalTime > 0.0 ? 1000.0 / stats.TotalTime : 0.0));
        ImGui::Spacing();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        ImGui::SliderInt("##sampling-rate", &stats.SamplingRate, 1, 10, "sampling rate: %d");
    }

    if (BeginSection("GC statistics", true))
    {
        ImGui::TextWrapped("native runtime\nno managed GC");
    }

    ImGui::End();

    if (settings.ShowImGuiDemo)
    {
        ImGui::ShowDemoWindow(&settings.ShowImGuiDemo);
    }
}

LaunchOptions ParseOptions(int argc, char** argv)
{
    LaunchOptions options;
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument == "--smoke")
        {
            options.FrameLimit = 5;
        }
        else if (argument == "--frames" && index + 1 < argc)
        {
            options.FrameLimit = std::max(1, std::atoi(argv[++index]));
        }
        else if (argument == "--demo" && index + 1 < argc)
        {
            options.DemoIndex = std::max(0, std::atoi(argv[++index]));
        }
        else if (argument == "--screenshot" && index + 1 < argc)
        {
            options.ScreenshotPath = argv[++index];
            if (options.FrameLimit == 0)
            {
                options.FrameLimit = 1;
            }
        }
        else if (argument == "--help" || argument == "-h")
        {
            std::printf("Usage: JitterDemo [--smoke] [--frames N] [--demo N] [--screenshot PATH]\n");
            std::exit(0);
        }
    }
    return options;
}

void GlfwErrorCallback(int error, const char* description)
{
    std::fprintf(stderr, "GLFW error %d: %s\n", error, description != nullptr ? description : "unknown");
}

} // namespace
