#include "stdlib.h"

inline int RandInt(int n) {
    return rand() % n;
}

inline int RandInt(size_t n) {
    return rand() % n;
}