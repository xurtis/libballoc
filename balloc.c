/*
 * An efficient block-based memory allocator.
 *
 * The implementation keeps track of a boubly linked list of blocks of
 * different sizes.
 *
 * Each block is known to be aligned to its size and the size of the
 * largest allocated block is also known.
 *
 *
 */

#include "balloc.h"
#include <assert.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Root block vector
 * =================
 */

/* Allocator for the root blocks */
static balloc_block_allocate_f *block_allocate;

/* A reference to a root block */
typedef struct {
	uintptr_t ref;
} blockref_t;

#define BLOCKREF_SIZE_BITS BALLOC_METADATA_SIZE_BITS
#define BLOCKREF_SIZE_MASK ((1 << BALLOC_METADATA_SIZE_BITS) - 1)
#define BLOCKREF_VADDR_MASK (~BLOCKREF_SIZE_MASK)

/* The initial blockref vector */
#define MIN_BLOCKREF_CAPACITY 31l
static blockref_t initial_blocks[MIN_BLOCKREF_CAPACITY];

/*
 * A vector containing the headers of all of the root blocks provided to
 * the allocator.
 *
 * This allows us to find the footer of the outermost block containing a
 * block to be freed in logn time. We can then use this to find the
 * footer of the freed block itself to determine its size and the type
 * of allocation.
 *
 * The root blocks are all distinct (i.e. the allocator rejects any
 * overlapping root blocks).
 */
static struct {
	blockref_t *blocks;
	size_t capacity;
	size_t count;
} root_blocks = {
	.blocks = initial_blocks,
	.capacity = MIN_BLOCKREF_CAPACITY,
	.count = 0,
};

/* Create a new block reference */
static inline blockref_t blockref_new(void *vaddr, size_t size);

/* Get the vaddr of a block reference */
static inline void *blockref_vaddr(blockref_t ref);

/* Get the log2 of the size of a block reference */
static inline size_t blockref_size(blockref_t ref);

/* Check if one blockref resides in another */
static inline bool blockref_contains(
	blockref_t outer,
	blockref_t inner
);

/* Check if a pair of blocks overlap */
static inline bool blockrefs_overlap(blockref_t b1, blockref_t b2);

/* The virtual address of after mergeing with adjacent block */
static inline void *blockref_merged_vaddr(blockref_t b);

/* Check if two blockrefs can be merged */
static inline bool blockrefs_mergeable(blockref_t b1, blockref_t b2);

/* Merge a block with its adjacent block */
static inline blockref_t blockref_merge(blockref_t block);

/*
 * Search for the index in the blockref vector to place a new block
 *
 * The index is one after the block with the highest starting address
 * before the block to be inserted.
 */
static size_t blockref_find_index(void *addr);

/* Allocate for internal data structures */
static void *internal_malloc(size_t size);

/* Allocate for internal data structures */
static void internal_free(void *allocation);

/* Insert a new element into the block vector */
static int blockref_insert_at(size_t index, blockref_t ref);

/* Allocate a new block and record it in the root vector */
static int alloc_new_root(
	balloc_metadata_t metadata,
	balloc_block_t *block
);

/*
 * Allocation lists
 * ================
 */

/* Footer data used in all memory objects */
#define FOOTER_SIZE 16ul
typedef struct {
	/* allocation metadata */
	struct {
		/* log2 of the size of the block */
		long size: BALLOC_METADATA_SIZE_BITS;
		/* The physical address and caching properties of the mapping
		 * are known */
		long device: 1;
		/* The memory is mapped cached */
		long cached: 1;
		/* The block has been allocated */
		long allocated: 1;
	} metadata;
	/* Physical address of the metadata block */
	uintptr_t paddr;
} alloc_footer_t;
_Static_assert(
	sizeof(alloc_footer_t) <= FOOTER_SIZE,
	"Footer larger than reserved size"
);

/* Get the footer of a block from it's base vaddr */
static inline alloc_footer_t *footer_from_base(
	void *vaddr,
	size_t size
);

/* Get the base of a block from its footer */
static inline void *base_from_footer(alloc_footer_t *footer);

/* Get the footer of a block from it's blockref */
static inline alloc_footer_t *footer_from_blockref(blockref_t ref);

/* Check that a pair of block footers can be merged */
static inline bool footers_mergeable(
	alloc_footer_t *f1,
	alloc_footer_t *f2
);

/* Header data used for free memory objects */
#define HEADER_SIZE 16ul
typedef struct alloc_header alloc_header_t;
struct alloc_header {
	/* Previous element in doubly-linked list */
	alloc_header_t *prev;
	/* Next element in doubly-linked list */
	alloc_header_t *next;
};
_Static_assert(
	sizeof(alloc_header_t) <= HEADER_SIZE,
	"Header larger than reserved size"
);

/* Min allocation is 64 bytes */
#define MIN_SIZE_BITS BLOCKREF_SIZE_BITS
/* Max allocation is 4GiB */
#define MAX_SIZE_BITS 32l
/* Number of sizes in each free list */
#define NUM_SIZES     (MAX_SIZE_BITS - MIN_SIZE_BITS)

/* Header and footer must fit in smallest allocation */
_Static_assert(
	(1 << MIN_SIZE_BITS) <= (HEADER_SIZE + FOOTER_SIZE) * 2,
	"Minimum allocation size too small for header and footer"
);

/* Non-device free lists */
static alloc_header_t *nondev_blocks[NUM_SIZES];

/* Cached device free lists */
static alloc_header_t *cached_blocks[NUM_SIZES];

/* Uncached device free lists */
static alloc_header_t *uncached_blocks[NUM_SIZES];

/*
 * Utilities
 */

static inline uintptr_t bit_size(size_t bit) {
	if (bit >= sizeof(uintptr_t)) {
		return 0;
	} else {
		return 1 << bit;
	}
}

static inline uintptr_t mask(size_t bit) {
	if (bit >= sizeof(uintptr_t)) {
		return ~0;
	} else {
		return (1 << bit) - 1;
	}
}

/*
 * Blockref implementation
 */

static inline blockref_t blockref_new(void *vaddr, size_t size) {
	return (blockref_t) {
		.ref = (
			(((uintptr_t)vaddr) & BLOCKREF_VADDR_MASK) |
			(size & BLOCKREF_SIZE_MASK)
		),
	};
}

static inline void *blockref_vaddr(blockref_t ref) {
	return (void *)(ref.ref & BLOCKREF_VADDR_MASK);
}

static inline size_t blockref_size(blockref_t ref) {
	return ref.ref & BLOCKREF_SIZE_MASK;
}

static inline bool blockref_contains(
	blockref_t outer,
	blockref_t inner
) {
	uintptr_t start = (uintptr_t)blockref_vaddr(outer);
	uintptr_t end = start + mask(blockref_size(outer));
	uintptr_t addr = (uintptr_t)blockref_vaddr(inner);
	return start <= addr && addr <= end;
}

static inline bool blockrefs_overlap(blockref_t b1, blockref_t b2) {
	/* Blocks cannot partially overlap by definition, they can only
	 * completely overlap. As such, it is sufficient to check either
	 * block contains the other */
	return blockref_contains(b1, b2) || blockref_contains(b2, b1);
}

static inline void *blockref_merged_vaddr(blockref_t b) {
	size_t merged_size = blockref_size(b) + 1;
	uintptr_t vaddr = (uintptr_t)blockref_vaddr(b);
	return (void *)(vaddr & ~mask(merged_size));
}

static inline bool blockrefs_mergeable(blockref_t b1, blockref_t b2) {
	void *b1_merged = blockref_merged_vaddr(b1);
	void *b2_merged = blockref_merged_vaddr(b2);
	return b1_merged == b2_merged;
}

static inline blockref_t blockref_merge(blockref_t block) {
	return blockref_new(
		blockref_merged_vaddr(block),
		blockref_size(block) + 1
	);
}

static size_t blockref_find_index(void *addr) {
	size_t low = 0;
	size_t high = root_blocks.count;
	uintptr_t vaddr = (uintptr_t)addr;

	/*
	 * invariants:
	 *  - All blocks < low have a vaddr <= the new block vaddr
	 *  - All blocks >= high have a vaddr > than the new block vaddr
	 *  - low <= high
	 *  - high <= root_blocks.count;
	 * measure:
	 *  - decreasing: high - low
	 */
	while (low != high) {
		/* Get the midpoint */
		size_t mid = low + ((high - low) / 2);
		blockref_t mid_block = root_blocks.blocks[mid];

		if ((uintptr_t)blockref_vaddr(mid_block) <= vaddr) {
			low = mid + 1;
		} else {
			high = mid;
		}
	}

	return low;
}


static void *internal_malloc(size_t size) {
	/* TODO: implement internal allocation */
	return NULL;
}

static void internal_free(void *allocation) {
	/* TODO: implement internal allocation */
	return;
}

static int blockref_insert_at(size_t index, blockref_t ref) {
	if (root_blocks.count + 1 >= root_blocks.capacity) {
		/* Allocate a bigger vector */
		blockref_t *blocks = internal_malloc(
			sizeof(blockref_t) * root_blocks.capacity * 2
		);
		if (blocks == NULL) {
			return BALLOC_OUT_OF_MEMORY;
		}

		/* Copy between the vectors */
		size_t src = 0;
		size_t dest = 0;

		root_blocks.count += 1;
		while (dest < root_blocks.count) {
			if (dest == index) {
				blocks[dest] = ref;
			} else {
				blocks[dest] = root_blocks.blocks[src];
				src += 1;
			}
			dest += 1;
		}

		/* Free the old vector */
		if (root_blocks.blocks != initial_blocks) {
			internal_free(root_blocks.blocks);
		}
		root_blocks.blocks = blocks;
	} else {
		/* Shift the tail of the vector up */
		size_t tail = root_blocks.count;
		root_blocks.count += 1;
		while (tail > index) {
			root_blocks.blocks[tail] = root_blocks.blocks[tail - 1];
			tail -= 1;
		}

		root_blocks.blocks[index] = ref;
	}

	return BALLOC_SUCCESS;
}

static int alloc_new_root(
	balloc_metadata_t metadata,
	balloc_block_t *block
) {
	if (block_allocate == NULL) {
		/* No block allocator set */
		return BALLOC_ALLOCATOR_NOT_SET;
	}

	balloc_block_t tmp = { .paddr = 0 };
	int err = block_allocate(metadata, &tmp);
	if (err != BALLOC_SUCCESS) {
		return err;
	}

	/* Check the requested metadata was satisfied */
	assert(tmp.metadata.size >= metadata.size);
	assert(tmp.metadata.device == metadata.device);
	if (tmp.metadata.device) {
		assert(tmp.metadata.cached == metadata.cached);
	}

	/* Write the footer to the block */
	alloc_footer_t *footer = footer_from_base(
		tmp.vaddr,
		tmp.metadata.size
	);
	footer->metadata.size = tmp.metadata.size;
	footer->metadata.cached = tmp.metadata.cached;
	footer->metadata.device = tmp.metadata.device;
	footer->metadata.allocated = false;
	footer->paddr = tmp.paddr;

	/* Find a location in the root block vector */
	size_t index = blockref_find_index(tmp.vaddr);
	blockref_t ref = blockref_new(tmp.vaddr, tmp.metadata.size);

	/* Ensure previous block dosen't contain new block */
	bool merge_with_previous = false;
	if (index > 0) {
		if (blockref_contains(root_blocks.blocks[index - 1], ref)) {
			return BALLOC_OVERLAPPING_BLOCK;
		} else {
			merge_with_previous = blockrefs_mergeable(
				ref,
				root_blocks.blocks[index - 1]
			);
			merge_with_previous = merge_with_previous &&
				footers_mergeable(
					footer_from_blockref(root_blocks.blocks[index - 1]),
					footer
				);
		}
	}

	/* Ensure new block dosen't contain next block */
	bool merge_with_next = false;
	if (index < root_blocks.count) {
		if (blockref_contains(ref, root_blocks.blocks[index])) {
			return BALLOC_OVERLAPPING_BLOCK;
		} else {
			merge_with_next = blockrefs_mergeable(
				ref,
				root_blocks.blocks[index]
			);
			/* Only merge if cached is same and contiguous physically if
			 * device */
			merge_with_next = merge_with_next &&
				footers_mergeable(
					footer,
					footer_from_blockref(root_blocks.blocks[index])
				);
		}
	}

	/* Try merging with the previous or next blocks before inserting */
	if (merge_with_previous) {
		root_blocks.blocks[index - 1] = blockref_merge(ref);
	} else if (merge_with_next) {
		root_blocks.blocks[index] = blockref_merge(ref);
	} else {
		/* Extend the vector */
		err = blockref_insert_at(index, ref);
		if (err != BALLOC_SUCCESS) {
			return err;
		}
	}

	*block = tmp;
	return BALLOC_SUCCESS;
}

static inline alloc_footer_t *footer_from_base(
	void *vaddr,
	size_t size
) {
	return (void *)(((uintptr_t)vaddr) + bit_size(size) - FOOTER_SIZE);
}

static inline void *base_from_footer(alloc_footer_t *footer) {
	return (void *)(
		((uintptr_t)footer)
		+ FOOTER_SIZE
		- bit_size(footer->metadata.size)
	);
}

static inline alloc_footer_t *footer_from_blockref(blockref_t ref) {
	return footer_from_base(blockref_vaddr(ref), blockref_size(ref));
}

/* Public implementation */

void balloc_set_block_allocator(balloc_block_allocate_f *allocator) {
	block_allocate = allocator;
}

/* Check that a pair of block footers can be merged */
static inline bool footers_mergeable(
	alloc_footer_t *f1,
	alloc_footer_t *f2
) {
	bool mergeable = f1->metadata.device == f2->metadata.device;
	mergeable = mergeable && f1->metadata.cached == f2->metadata.cached;

	if (mergeable && f1->metadata.device) {
		uintptr_t vaddr1 = (uintptr_t)base_from_footer(f1);
		uintptr_t vaddr2 = (uintptr_t)base_from_footer(f2);
		mergeable = mergeable &&
			(vaddr2 - vaddr1) == (f2->paddr - f1->paddr);
	}

	return mergeable;
}
