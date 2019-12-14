# Parallel Motion Planning Library
This folder contains essential files for a parallel motion planning algorithm.
Due to parallelization, I hope I can make my algorithm much faster.
The language is C++ since other languages are not really designed for high concurrency.


# next steps
Find a reliable package that supports lock-free list manipulation or implement CAS myself.
Potential libraries include [https://github.com/khizmax/libcds], [https://github.com/rigtorp/awesome-lockfree]
But reading source code at [https://github.com/jeffi/prrts-c/blob/master/src/prrts.c] may also be a good idea.

This post seems cool, lock-free is not wait-free [http://concurrencyfreaks.blogspot.com/2014/10/linked-lists-with-wait-free-reads-in-c.html]
Also a cool github project [https://github.com/pramalhe/ConcurrencyFreaks]
A cool blog [https://www.memsql.com/blog/common-pitfalls-in-writing-lock-free-algorithms/]

# logs
On Nov 18, I was able to write code for a thread-safe linked list which will be heavily used in the tree.
I think I am learning to understand how this atomic and compare_exchange_weak things work.
I also made a serial KD-Tree work by modifying Kris' code.
Parallel version is being worked on.

On Dec 9, I already made a prrtstar work on some simple problems. However, the speedup is not as satisfactory as expected.
I shifted my focus a little bit on ao-rrt and made a basic version work.

On Dec 14, we wrote code to check if the tree is correct, i.e. a node's child's parent is the node itself, and the cost sum matches. Thank Yifan for writing those.