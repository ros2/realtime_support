rttest is a minimal tool for instrumenting and running tests for synchronous real-time systems.
It provides utilities for measuring and plotting jitter, latency, and missed deadlines.
It also provides a library with macros for instrumenting code.
It is designed with real-time Linux-based systems in mind, such as RTLinux/RT Preempt kernel.

Command line arguments:

-u --update-period: Specify the update period. Default units are microseconds. Use the suffix "s" for seconds, "ms" for milliseconds, "us" for microseconds, and "ns" for nanoseconds. Default update period will be 1ms.

-m --memory-size: If enabled, disable dynamic memory allocation and lock the stack to the specified size. "b" for bytes, "kb" for kilobytes, "mb" for megabytes, "gb" for gigabytes (use with caution). Default stack size will be 1024MB.

-i --iterations: Run the test for this many iterations. Defaults to infinite.

-p --plot: Enable plotting.

-tp --thread-priority: Set the thread priority of all threads launched by the test program. Individual thread priority can be set using the RTTEST_SET_PRIORITY macro.

-r --repeat: Repeat tests
