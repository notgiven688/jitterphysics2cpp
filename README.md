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
| C# | Release | 28 | 4.185 ms | 239 | 4.183 ms | 1.00x |
| C++ | `-O2` | 28 | 4.757 ms | 210 | 4.749 ms | 0.88x |
| C++ | `-O3` | 28 | 4.673 ms | 214 | 4.667 ms | 0.90x |
| C++ | `-O3 -march=native` | 28 | 4.324 ms | 231 | 4.317 ms | 0.97x |

#### Single-Threaded

| Implementation | Build | Wall avg | FPS | DebugTimings avg | Relative to C# |
| --- | --- | ---: | ---: | ---: | ---: |
| C# | Release | 46.321 ms | 22 | 46.316 ms | 1.00x |
| C++ | `-O2` | 55.068 ms | 18 | 55.067 ms | 0.84x |
| C++ | `-O3` | 55.975 ms | 18 | 55.974 ms | 0.83x |
| C++ | `-O3 -march=native` | 47.026 ms | 21 | 47.025 ms | 0.99x |

### Rotating Cube

Rotating hollow cube scene, deactivation enabled, 8001 dynamic bodies plus
one kinematic compound cube body.

#### Multithreaded

| Implementation | Build | Threads reported | Wall avg | FPS | DebugTimings avg | Relative to C# |
| --- | --- | ---: | ---: | ---: | ---: | ---: |
| C# | Release | 28 | 5.105 ms | 196 | 5.103 ms | 1.00x |
| C++ | `-O2` | 28 | 4.772 ms | 210 | 4.765 ms | 1.07x |
| C++ | `-O3` | 28 | 4.590 ms | 218 | 4.583 ms | 1.11x |
| C++ | `-O3 -march=native` | 28 | 4.576 ms | 219 | 4.568 ms | 1.12x |

#### Single-Threaded

| Implementation | Build | Wall avg | FPS | DebugTimings avg | Relative to C# |
| --- | --- | ---: | ---: | ---: | ---: |
| C# | Release | 32.804 ms | 30 | 32.800 ms | 1.00x |
| C++ | `-O2` | 42.609 ms | 23 | 42.608 ms | 0.77x |
| C++ | `-O3` | 42.299 ms | 24 | 42.298 ms | 0.78x |
| C++ | `-O3 -march=native` | 37.002 ms | 27 | 37.001 ms | 0.89x |

## License

This project follows the license of the original Jitter Physics 2 project.
