# Engine Deviations from Jitter2 2.8.8

This file lists relevant engine-level differences between the C# reference
implementation and this C++ port. It intentionally does not track demo,
benchmark, build-system, or renderer differences.

The goal of the port is behavioral parity, but some API and implementation
details differ because C++ has different ownership, dispatch, and callback
models.

## API and Type-System Differences

- `RigidBody.Tag` is represented as `std::any` instead of C# `object`.
- Public collection views are exposed as `std::vector` references or `std::span`
  views instead of C# collection interfaces/spans.
- Shape and body ownership is explicit. The engine stores world-owned bodies,
  constraints, and arbiters with C++ ownership types, while shapes passed to
  bodies are non-owning references/pointers unless a higher-level owner keeps
  them alive.
- C# generic virtual query methods such as
  `IDistanceTestable.Distance<T>` and `ISweepTestable.Sweep<T>` are represented
  with non-template virtual dispatch taking `ISupportMappable&`. C++ has no
  direct equivalent for virtual generic methods.
- C# delegate equality is not available for arbitrary `std::function` objects.
  World callback lists such as `PreStep`, `PreSubStep`, and `PostStep` therefore
  support removal by token instead of delegate subtraction.
- `RigidBody.BeginCollide` and `RigidBody.EndCollide` are single
  `std::function` callback slots rather than multicast C# events.

## Ownership and Lifetime

- Soft bodies unregister their `PostStep` callback explicitly through a stored
  callback token. This replaces the C# delegate unsubscribe pattern.
- Dynamic-tree proxies, shapes, constraints, and callback subscribers must not
  outlive the objects they reference. The C# implementation can rely on managed
  object lifetimes in places where the C++ port requires explicit ownership by
  the caller or containing object.
- `ConvexHullShape`, `PointCloudShape`, and related support-map data use shared
  internal storage where needed to make value-style cloning practical in C++.

## Concurrency

- `PairHashSet::ConcurrentAdd` uses a read-lock plus atomic slot insertion path.
  It intentionally omits the C# pre-lock duplicate fast path because C++
  `std::vector` storage can be invalidated by concurrent resize, while the C#
  code works with replaced managed array references.
- The C++ `ThreadPool` is implemented with `std::thread`, mutexes, condition
  variables, and explicit shutdown. It follows the C# scheduling model, but its
  thread lifetime and wake/pause mechanics are native C++ constructs.
- Thread-local pools are used for reusable arbiters and island-helper scratch
  storage where the C# implementation uses managed/thread-static storage.

## Diagnostics

- The logger preserves C#-style numbered placeholder formatting, but listener
  registration is represented with C++ callable objects.
- `Tracer` writes Chrome trace-compatible output like the C# version, but uses
  C++ stream/file lifetime and RAII helpers internally.

## Exceptions

- The C++ port maps C# exception behavior to standard C++ exception types such
  as `std::invalid_argument`, `std::out_of_range`, `std::logic_error`, and
  `std::runtime_error`. Exact exception class names therefore differ from C#.
