#ifndef HARLIEALLOC_H
#define HARLIEALLOC_H

#include <stdlib.h>

void * hMalloc(size_t size);
void hFree(void * free);

// Note: not implemented

/*
 * This is c++
 * THis will greatly increass debug outputs!!!!!
//void* operator new(size_t size) throw(std::bad_alloc);
//void* operator new[](size_t size) throw(std::bad_alloc);
//void operator delete(void* needsFreed) throw();
//void operator delete[](void* needsFreed) throw();
 */

#endif
