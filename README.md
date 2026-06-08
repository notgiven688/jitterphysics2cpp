# Jitter2 C++

This repository is an automatic AI port by OpenAI Codex of
[Jitter Physics 2](https://github.com/notgiven688/jitterphysics2) version 2.8.8
from C# to C++.

The purpose of this port is performance comparison against the original managed
implementation. It is not a hand-written rewrite; the C# version remains the
reference implementation.

## Projects

This repository contains three main projects:

| Project | Description |
| --- | --- |
| `Jitter2` | C++ physics engine library. |
| `JitterDemo` | ImGui/OpenGL demo application. |
| `JitterTests` | C++ tests for engine behavior and parity checks. |

`JitterBenchmark` is also included for performance experiments.
Matching C++ and C# benchmark programs live under `Benchmarks`.

## Build

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

Run tests:

```sh
ctest --test-dir build --output-on-failure
```

Run the demo:

```sh
./build/JitterDemo/JitterDemo
```

Run benchmarks:

```sh
./build/Benchmarks/Cpp/JitterBenchmark
dotnet run -c Release --project Benchmarks/CSharp/JitterBenchmark.CSharp.csproj
```

Use double precision:

```sh
cmake -S . -B build-double -DCMAKE_BUILD_TYPE=Release -DJITTER_DOUBLE_PRECISION=ON
cmake --build build-double -j
```

## Benchmarks

Local snapshot, single precision, two demo-derived scenes, default `dt = 0.01`.
Each entry is the median of 3 runs using `--warmup 300 --frames 1000`; values
in parentheses show min-max.

Environment:

- CPU: AMD Ryzen 9 7950X, 16 cores / 32 threads
- OS: Ubuntu 24.04
- C++: GCC 13.3.0
- C#: Jitter2 NuGet 2.8.8, .NET 10.0 runtime via `net8.0` roll-forward

Lower frame time is better.

### Colosseum

Large stacked colosseum scene, deactivation disabled, 8818 dynamic bodies plus
static floor.

#### Multithreaded

| Implementation | Build | Threads reported | Wall avg | FPS | DebugTimings avg | Relative to C# |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| C# | Release | 28 | 3.846 ms (3.775-3.928) | 260 | 3.845 ms (3.774-3.926) | 1.00x |
| C++ | `-O2` | 28 | 4.462 ms (4.439-4.524) | 224 | 4.455 ms (4.431-4.517) | 0.86x |
| C++ | `-O3` | 28 | 4.419 ms (4.407-4.444) | 226 | 4.411 ms (4.399-4.436) | 0.87x |
| C++ | `-O3 -march=native` | 28 | 4.096 ms (4.017-4.138) | 244 | 4.089 ms (4.009-4.130) | 0.94x |

#### Single-Threaded

| Implementation | Build | Wall avg | FPS | DebugTimings avg | Relative to C# |
| --- | --- | ---: | ---: | ---: | ---: |
| C# | Release | 46.706 ms (45.642-46.867) | 21 | 46.703 ms (45.640-46.864) | 1.00x |
| C++ | `-O2` | 54.680 ms (54.479-55.037) | 18 | 54.679 ms (54.478-55.036) | 0.85x |
| C++ | `-O3` | 54.222 ms (54.067-54.604) | 18 | 54.221 ms (54.066-54.604) | 0.86x |
| C++ | `-O3 -march=native` | 47.701 ms (47.434-48.115) | 21 | 47.701 ms (47.433-48.115) | 0.98x |

### Rotating Cube

Rotating hollow cube scene, deactivation enabled, 8001 dynamic bodies plus
one kinematic compound cube body.

#### Multithreaded

| Implementation | Build | Threads reported | Wall avg | FPS | DebugTimings avg | Relative to C# |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| C# | Release | 28 | 5.820 ms (5.702-6.073) | 172 | 5.819 ms (5.701-6.071) | 1.00x |
| C++ | `-O2` | 28 | 6.320 ms (6.317-6.435) | 158 | 6.312 ms (6.309-6.427) | 0.92x |
| C++ | `-O3` | 28 | 6.402 ms (6.322-6.405) | 156 | 6.394 ms (6.314-6.397) | 0.91x |
| C++ | `-O3 -march=native` | 28 | 6.225 ms (6.135-6.370) | 161 | 6.217 ms (6.127-6.362) | 0.94x |

#### Single-Threaded

| Implementation | Build | Wall avg | FPS | DebugTimings avg | Relative to C# |
| --- | --- | ---: | ---: | ---: | ---: |
| C# | Release | 51.930 ms (50.430-52.053) | 19 | 51.927 ms (50.428-52.051) | 1.00x |
| C++ | `-O2` | 75.449 ms (73.929-75.551) | 13 | 75.448 ms (73.928-75.550) | 0.69x |
| C++ | `-O3` | 70.847 ms (70.762-75.992) | 14 | 70.846 ms (70.761-75.992) | 0.73x |
| C++ | `-O3 -march=native` | 66.435 ms (66.364-68.108) | 15 | 66.435 ms (66.364-68.107) | 0.78x |

## License

This project follows the license of the original Jitter Physics 2 project.
