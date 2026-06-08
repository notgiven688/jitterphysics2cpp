# Benchmarks

This directory contains matching C++ and C# benchmark programs for comparing
the C++ port with the original Jitter2 NuGet package.

## C++

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DJITTER_BUILD_BENCHMARKS=ON
cmake --build build --target JitterBenchmark -j
./build/Benchmarks/Cpp/JitterBenchmark
```

## C#

```sh
dotnet run -c Release --project Benchmarks/CSharp/JitterBenchmark.CSharp.csproj
```

Both programs accept:

```text
--scene colosseum|rotating-cube --frames N --warmup N --threads N --single-thread --dt seconds
```
