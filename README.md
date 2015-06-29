#rttest

rttest is a minimal tool for instrumenting and running tests for synchronous real-time systems.
It provides utilities for measuring and plotting jitter, latency, and missed deadlines.
It also provides a library with macros for instrumenting code.
It is designed with real-time Linux-based systems in mind, such as RTLinux/RT Preempt kernel.

## Build instructions
Build from source:

```
mkdir build
cd build

cmake ..
sudo make install
```

Alternatively for a local install:

```
cmake .. -DCMAKE_INSTALL_PREFIX=<build folder>
make install
```

Build and run the example:

```
cd examples
mkdir build
cd build
cmake .. (-DCMAKE_INSTALL_PREFIX=<build folder>)
make
./example_loop
```

## Command line arguments
Passing `argc` and `argv` of an instrumented main function to `rttest_read_args` will enable command line arguments for the instrumented function.

-u Specify the update period. Default units are microseconds. Use the suffix "s" for seconds, "ms" for milliseconds, "us" for microseconds, and "ns" for nanoseconds. Default update period will be 1ms

-m If enabled, enable dynamic memory allocation and lock the stack to the specified size. "b" for bytes, "kb" for kilobytes, "mb" for megabytes, "gb" for gigabytes (use with caution). Default stack size will be 1024MB.

-i Specify how many iterations to run the real-time loop.

-p Enable plotting.

-tp Set the thread priority of all threads launched by the test program. Individual thread priority can be set using the `RTTEST_SET_PRIORITY` macro.

-r Repeat tests (not yet implemented)

## Issues (to be transferred to tracker)

* Add gtest
* Fix timestamp precision output in data
* Implement reps options
* Implement asynchronous samples
* count and plot page faults/cache misses
