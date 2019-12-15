#ifndef ALLOC_H
#define ALLOC_H

#include <stdlib.h>
#include <pthread.h>

/* 64 is a fairly standard cache-line size in x86, but this should be
 * parameterized elsewhere. */
#define CACHE_LINE_SIZE 64

#define ALIGN_UP(x, b) (((size_t)(x) + (b-1)) & ~((size_t)(b-1)))
#define CACHE_ALIGN(x) ALIGN_UP(x, CACHE_LINE_SIZE)


static inline int
aligned_offset(void *ptr, size_t b)
{
        size_t address = (size_t)ptr;
        return (ALIGN_UP(address, b) - address);
}




typedef struct tl_mempool {
        pthread_key_t key;
        size_t chunk_size;
} tl_mempool_t;

#define struct_alloc(type) ((type *)malloc(sizeof(type)))
#define array_alloc(type, count) ((type *)malloc(sizeof(type) * count))
#define array_grow(array, count) ((typeof(array))realloc(array, sizeof(array[0]) * count));
#define array_copy(array, count) memdup(array, sizeof(array[0]) * count)

int tl_mempool_init(tl_mempool_t *pool, size_t chunk_size);
int tl_mempool_destroy(tl_mempool_t *pool);
void *tl_alloc(tl_mempool_t *pool, size_t size);
void tl_free(tl_mempool_t *pool, void *ptr);

void *memdup(const void *source, size_t bytes);

#endif /* ALLOC_H */
