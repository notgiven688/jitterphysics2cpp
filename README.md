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

Local snapshot, single precision, two demo-derived scenes,
`--warmup 120 --frames 300`, default `dt = 0.01`.

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
| C# | Release | 28 | 4.168 ms | 240 | 4.165 ms | 1.00x |
| C++ | `-O2` | 28 | 4.751 ms | 210 | 4.744 ms | 0.88x |
| C++ | `-O3` | 28 | 4.727 ms | 212 | 4.720 ms | 0.88x |
| C++ | `-O3 -march=native` | 28 | 4.396 ms | 227 | 4.389 ms | 0.95x |

#### Single-Threaded

| Implementation | Build | Wall avg | FPS | DebugTimings avg | Relative to C# |
| --- | --- | ---: | ---: | ---: | ---: |
| C# | Release | 46.003 ms | 22 | 45.999 ms | 1.00x |
| C++ | `-O2` | 54.245 ms | 18 | 54.244 ms | 0.85x |
| C++ | `-O3` | 54.917 ms | 18 | 54.916 ms | 0.84x |
| C++ | `-O3 -march=native` | 47.330 ms | 21 | 47.329 ms | 0.97x |

### Rotating Cube

Rotating hollow cube scene, deactivation enabled, 8001 dynamic bodies plus
one kinematic compound cube body.

#### Multithreaded

| Implementation | Build | Threads reported | Wall avg | FPS | DebugTimings avg | Relative to C# |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| C# | Release | 28 | 5.430 ms | 184 | 5.427 ms | 1.00x |
| C++ | `-O2` | 28 | 4.720 ms | 212 | 4.713 ms | 1.15x |
| C++ | `-O3` | 28 | 4.739 ms | 211 | 4.732 ms | 1.15x |
| C++ | `-O3 -march=native` | 28 | 4.650 ms | 215 | 4.643 ms | 1.17x |

#### Single-Threaded

| Implementation | Build | Wall avg | FPS | DebugTimings avg | Relative to C# |
| --- | --- | ---: | ---: | ---: | ---: |
| C# | Release | 32.386 ms | 31 | 32.382 ms | 1.00x |
| C++ | `-O2` | 43.161 ms | 23 | 43.160 ms | 0.75x |
| C++ | `-O3` | 42.218 ms | 24 | 42.217 ms | 0.77x |
| C++ | `-O3 -march=native` | 37.843 ms | 26 | 37.842 ms | 0.86x |

## License

This project follows the license of the original Jitter Physics 2 project.
