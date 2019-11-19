# Parallel Motion Planning Library
This folder contains essential files for a parallel motion planning algorithm.
Due to parallelization, I hope I can make my algorithm much faster.
The language is C++ since other languages are not really designed for high concurrency.


# next steps
Find a reliable package that supports lock-free list manipulation or implement CAS myself.
Potential libraries include [https://github.com/khizmax/libcds], [https://github.com/rigtorp/awesome-lockfree]
But reading source code at [https://github.com/jeffi/prrts-c/blob/master/src/prrts.c] may also be a good idea.

# logs
On Nov 18, I was able to write code for a thread-safe linked list which will be heavily used in the tree.
I think I am learning to understand how this atomic and compare_exchange_weak things work.
I also made a serial KD-Tree work by modifying Kris' code.
Parallel version is being worked on.