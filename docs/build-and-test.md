# Build and Test

## Initialize Submodules

```bash
git submodule update --init --recursive
```

## Configure

```bash
cmake --preset clang-debug
```

## Build

```bash
cmake --build --preset build-clang-debug
```

## Test

```bash
ctest --preset test-clang-debug
```

## Optional Sanitizers

```bash
cmake --preset clang-debug-asan-ubsan
cmake --build --preset build-clang-debug-asan-ubsan
ctest --preset test-clang-debug-asan-ubsan
```

If configuration fails with a GoogleTest error, initialize submodules and retry.
