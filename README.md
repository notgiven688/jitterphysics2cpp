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
| C# | Release | 28 | 3.832 ms | 261 | 3.830 ms | 1.00x |
| C++ | `-O2` | 28 | 4.489 ms | 223 | 4.482 ms | 0.85x |
| C++ | `-O3` | 28 | 4.465 ms | 224 | 4.458 ms | 0.86x |
| C++ | `-O3 -march=native` | 28 | 4.139 ms | 242 | 4.133 ms | 0.93x |

#### Single-Threaded

| Implementation | Build | Wall avg | FPS | DebugTimings avg | Relative to C# |
| --- | --- | ---: | ---: | ---: | ---: |
| C# | Release | 40.363 ms | 25 | 40.360 ms | 1.00x |
| C++ | `-O2` | 45.044 ms | 22 | 45.044 ms | 0.90x |
| C++ | `-O3` | 44.437 ms | 23 | 44.436 ms | 0.91x |
| C++ | `-O3 -march=native` | 39.498 ms | 25 | 39.498 ms | 1.02x |

### Rotating Cube

Rotating hollow cube scene, deactivation enabled, 8001 dynamic bodies plus
one kinematic compound cube body.

#### Multithreaded

| Implementation | Build | Threads reported | Wall avg | FPS | DebugTimings avg | Relative to C# |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| C# | Release | 28 | 4.870 ms | 205 | 4.868 ms | 1.00x |
| C++ | `-O2` | 28 | 4.855 ms | 206 | 4.847 ms | 1.00x |
| C++ | `-O3` | 28 | 4.754 ms | 210 | 4.747 ms | 1.02x |
| C++ | `-O3 -march=native` | 28 | 4.763 ms | 210 | 4.756 ms | 1.02x |

#### Single-Threaded

| Implementation | Build | Wall avg | FPS | DebugTimings avg | Relative to C# |
| --- | --- | ---: | ---: | ---: | ---: |
| C# | Release | 26.051 ms | 38 | 26.050 ms | 1.00x |
| C++ | `-O2` | 31.972 ms | 31 | 31.971 ms | 0.81x |
| C++ | `-O3` | 31.351 ms | 32 | 31.351 ms | 0.83x |
| C++ | `-O3 -march=native` | 29.013 ms | 34 | 29.013 ms | 0.90x |

In these runs, C# and C++ are close in the multithreaded cases. The rotating
cube scene is effectively tied when multithreaded; colosseum favors C# slightly
unless native CPU tuning is enabled. Single-threaded C++ catches up in the
colosseum scene with `-march=native`; rotating cube remains faster in C#.

Additional useful comparisons would be double precision, fixed worker counts,
C++ LTO/IPO, Clang builds, C# ReadyToRun/PGO, and longer repeated runs with
min/median/max instead of a single snapshot.

## License

This project follows the license of the original Jitter Physics 2 project.
