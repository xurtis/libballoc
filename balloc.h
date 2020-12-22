/*
 * An efficient block-based memory allocator.
 *
 * The allocator manages a set of virtually-mapped blocks of memory
 * including blocks of memory used for DMA. Each block is a power of 2
 * size that is aligned to its size.
 *
 * This implementation also guarantees that all allocations it produces
 * are aligned to the smallest power of 2 greater than the allocation
 * size.
 *
 * This interface would be innappropriate for allocation over a shared
 * pool of memory (if each user of the memory were to use its own
 * allocator).
 *
 * This library is also not thread-safe. For safe use across multiple
 * threads, all access must be guarded by a mutex.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* Space in metadata used to indicate size of block */
#define BALLOC_METADATA_SIZE_BITS 6l

/* Metadata describing a block allocation */
typedef struct {
	/* log2 of the size of the block */
	long size: BALLOC_METADATA_SIZE_BITS;
	/* The physical address and caching properties of the mapping
	 * are known */
	long device: 1;
	/* The memory is mapped cached */
	long cached: 1;
} balloc_metadata_t;

/* A block provided to the allocator for use in allocation */
typedef struct {
	balloc_metadata_t metadata;
	/* The virtual address of the block of memory */
	void *vaddr;
	/* The physical address of the memory */
	uintptr_t paddr;
} balloc_block_t;

/*
 * An implementation of a memory block provider
 *
 * The block returned will be a power 2 in size no less than the log2
 * size described in the metadata. Both the virtual and physical
 * addresses will be aligned to the size of the returned block.
 *
 * If the requested metadata has device set, the returned block
 * must have a physical address associated and must indicate whether the
 * block is mapped cached.
 *
 * If the requested metadata has cached set, the returned block
 * must be mapped with a cached mapping, otherwise it must be mapped
 * uncached.
 *
 * Must return one of the error status values below.
 */
typedef int balloc_block_allocate_f(
	/* Requirements of the block to be mapped */
	balloc_metadata_t metadata,
	/* Block returned from allocator */
	balloc_block_t *block
);

/* Block allocator implementaiton errors */
enum {
	/* Successfully allocated block */
	BALLOC_SUCCESS = 0,
	/* No allocator set */
	BALLOC_ALLOCATOR_NOT_SET = -1024,
	/* Out of memory */
	BALLOC_OUT_OF_MEMORY = -1025,
	/* Allocator produced memory overlapping with existing region */
	BALLOC_OVERLAPPING_BLOCK = -1026,
};

/* Set the block provider for balloc.
 *
 * Blocks are never returned to the block provider and the block
 * provider may be swapped out at any time.
 */
void balloc_set_block_allocator(balloc_block_allocate_f *allocator);

/* See `malloc(3)` */
void *balloc_malloc(size_t size);

/* See `free(3)`; works on any allocations provided by balloc. */
void balloc_free(void *allocation);

/* See `calloc(3)` */
void *balloc_calloc(size_t nmemb, size_t size);

/* See `realloc(3)` */
void *balloc_realloc(void *allocation, size_t size);

/* Allocate a block of memory for use with a device */
void *balloc_device(size_t size, bool cached);
