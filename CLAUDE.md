# CLAUDE.md — cuBQL

## Project Overview

cuBQL ("cubicle") is an NVIDIA open-source CUDA/C++ library for GPU-accelerated construction and traversal of Bounding Volume Hierarchies (BVHs). It provides templated, header-only spatial data structures supporting arbitrary scalar types (`int`, `float`, `double`, `long long`) and dimensions (2D, 3D, 4D). The library supports both binary and wide (N-ary) BVH variants with multiple build strategies and lambda-based traversal for custom queries.

**Version**: 1.3.0  
**License**: Apache-2.0  
**Language**: C++17 / CUDA

## Architecture

```
cuBQL/
├── cuBQL/                    # Core library (header-only)
│   ├── bvh.h                 # Core types: BinaryBVH, WideBVH, BuildConfig
│   ├── math/                 # Vector, box, ray, affine transforms
│   │   ├── vec.h             # vec_t<T,D> template + aliases (vec3f, etc.)
│   │   ├── box.h             # box_t<T,D> axis-aligned bounding box
│   │   └── common.h          # Portability macros (__cubql_both, CUBQL_CUDA_CALL, etc.)
│   ├── builder/
│   │   ├── cuda.h            # GPU builder entry point: gpuBuilder()
│   │   ├── cuda/             # GPU build algorithms (spatial median, SAH, ELH, Morton, wide)
│   │   ├── cpu.h             # CPU builder entry point: cpuBuilder()
│   │   ├── cpu/              # CPU build algorithms
│   │   └── omp/              # OpenMP-parallelized builders
│   ├── traversal/            # Generic traversal templates (lambda-based)
│   │   ├── shrinkingRadiusQuery.h   # Nearest-neighbor style queries
│   │   ├── fixedRadiusQuery.h       # Fixed-radius range queries
│   │   ├── rayQueries.h            # Ray intersection queries
│   │   └── ...
│   └── queries/              # Pre-built query implementations
│       ├── pointData/        # Point cloud queries (findClosest, knn)
│       └── triangleData/     # Triangle mesh queries (closest point, inside/outside, line-of-sight)
├── samples/                  # 7 example programs (s01–s07)
│   └── common/               # Shared utilities (OBJ loader, CLI, data generators)
└── testing/                  # Test directory (exists but currently disabled in CMake)
```

**Key relationships**:
- `builder/` produces BVH structures defined in `bvh.h` from bounding box arrays
- `traversal/` provides generic tree-walk templates that accept user lambdas
- `queries/` implements specific spatial queries using `traversal/` templates
- All types are parameterized on `<T, D>` (scalar type, dimension count)

**Two BVH types**:
- `BinaryBVH<T,D>` (`bvh_t`): 2-child nodes, 16-byte aligned. Node stores box + 48-bit offset + 16-bit count. Leaves have count > 0.
- `WideBVH<T,D,N>`: N-ary nodes with validity flags per child.

## Build & Setup

**Requirements**: CMake 3.16+, C++17, CUDA 12+ (optional — builds host-only without CUDA)

```bash
# Standard build
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
cmake --build .

# Specify CUDA architectures (required for CMake < 3.24)
cmake .. -DCMAKE_CUDA_ARCHITECTURES=80

# CPU-only (disable CUDA)
cmake .. -DCUBQL_DISABLE_CUDA=ON

# OpenMP builders
cmake .. -DCUBQL_OMP=ON
```

**As a subproject** (most common usage):
```cmake
# MUST set architectures before adding cuBQL
set(CMAKE_CUDA_ARCHITECTURES "native")  # or "all-major", or "80"
add_subdirectory(path/to/cuBQL)
target_link_libraries(myapp cuBQL_cuda_float3)  # link type-specific target
```

**Library targets** — for each type combo `{int,float,double,longlong}` x `{2,3,4}`:
- `cuBQL_cuda_<type><dim>` / `cuBQL_cuda_<type><dim>_static` — GPU builders
- `cuBQL_cpu_<type><dim>` / `cuBQL_cpu_<type><dim>_static` — CPU builders
- `cuBQL` — header-only interface target (no pre-compiled builders)

Targets use `EXCLUDE_FROM_ALL`; only linked targets are compiled.

**Header-only mode** (alternative to linking pre-compiled targets):
```cpp
#define CUBQL_GPU_BUILDER_IMPLEMENTATION 1
#include <cuBQL/builder/cuda.h>
```

## Development Workflow

Samples are built automatically in standalone mode (not when cuBQL is a subproject). After building:

```bash
# Run a sample (from build directory)
./s01_closestPoint_points_gpu
./s02_distanceToTriangleMesh --mesh path/to/model.obj
```

Samples demonstrate the typical usage pattern:
1. Generate/load primitives and compute bounding boxes
2. Call `cuBQL::cuda::gpuBuilder(bvh, boxes, N)` or `cuBQL::cpu::cpuBuilder(...)`
3. Launch queries using traversal templates with custom lambdas
4. Free BVH with `cuBQL::cuda::free(bvh)` or `cuBQL::cpu::freeBVH(bvh)`

## Testing

Testing infrastructure exists (`testing/` directory) but is **currently disabled** in the top-level CMakeLists.txt (line 149 is commented out). No formal test framework is configured. The samples serve as functional verification.

CI runs on push/PR to `main`:
- **Ubuntu**: CUDA 13.0.2/GCC 13 (Release + Debug), CUDA 12.0.0/GCC 11 (Release)
- **Windows**: separate workflow

## Key Commands

```bash
# Configure + build
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build

# Build specific target only
cmake --build build --target cuBQL_cuda_float3

# Install
cmake --install build --prefix /path/to/install

# Clean rebuild
rm -rf build && cmake -B build && cmake --build build
```

## Code Conventions

**Naming**:
- Types use `_t` suffix: `vec_t<T,D>`, `box_t<T,D>`, `bvh_t<T,D>`
- Shorthand aliases: `vec3f`, `vec3i`, `box3f`, `bvh3f`
- Macros prefixed `CUBQL_`: `CUBQL_ALIGN()`, `CUBQL_CUDA_CALL()`, `CUBQL_INF`
- Portability attributes: `__cubql_both`, `__cubql_host`, `__cubql_device`
- Descriptive lambda params: `queryPoint`, `sqrMaxSearchRadius`, `lambdaToExecuteForEachCandidate`

**Namespaces**:
- All code under `cuBQL::`
- Backends: `cuBQL::cuda::`, `cuBQL::cpu::`, `cuBQL::omp::`
- Query domains: `cuBQL::points::`, `cuBQL::triangles::`
- Traversal: `cuBQL::shrinkingRadiusQuery::`, `cuBQL::fixedRadiusQuery::`
- Internal: `gpuBuilder_impl::`, `sahBuilder_impl::`

**File headers**: every file starts with SPDX Apache-2.0 copyright block.

**Template pattern** — types are consistently parameterized:
```cpp
template<typename T, int D>
struct MyType {
  using scalar_t = T;
  using vec_t = cuBQL::vec_t<T,D>;
  using box_t = cuBQL::box_t<T,D>;
};
```

**Commits**: require sign-off (`git commit -s`) per DCO.

## Important Context

**Squared distances everywhere**: All traversal templates and queries use **squared** distances/radii. Lambda callbacks receive and return `sqrRadius`, not radius. Getting this wrong silently produces incorrect results.

**Memory management**: GPU builders allocate device memory; CPU builders use `new[]`. Always match the correct free function (`cuda::free()` vs `cpu::freeBVH()`). GPU memory allocation defaults to `cudaMallocAsync` (CUDA 11.2+) with fallback to managed memory.

**Invalid primitives**: Boxes with `lower.x > upper.x` are treated as invalid — excluded from BVH construction but do not cause errors. NaN/denorm behavior is undefined.

**CUDA architecture mismatch**: When using cuBQL as a subproject, you **must** set `CMAKE_CUDA_ARCHITECTURES` before `add_subdirectory(cuBQL)` or the build will fail with a detailed error message. This is an intentional guard.

**Build algorithms**:
- Spatial median (default) — fast, reasonable quality
- SAH — slower build, better traversal quality
- ELH (edge-length heuristic) — experimental
- Morton/radix — fastest build, lower quality
- Each selectable via `BuildConfig::buildMethod`

**Wide BVH**: `WideBVH<T,D,N>` uses N-ary branching with per-child validity flags. Wide BVH builders and queries exist but are less mature than binary BVH.

**HIP support**: Portability macros in `common.h` abstract CUDA/HIP, but HIP is not actively tested in CI.
