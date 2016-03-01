#include "harliealloc.h"
#include "harlielog.h"
#include "string.h"

void * hMalloc(size_t size)
{
    void * location;
    location = malloc(size);
    bzero(location, size);
    LOG.MEMORY("Just malloced %i at %p",size,location);
    return location;
}

void hFree(void * needsFreed)
{
    LOG.MEMORY("About to free location : %p",needsFreed);
    free(needsFreed);
}

// NOTE: seems like code that was to make it easier to alloc and free memory

/*
void* operator new(size_t size) throw(std::bad_alloc)
{
    void * location;
    location = malloc(size);
    LOG.MEMORY("Just malloced using new %i at %p",size,location);
    return location;
}

void* operator new[](size_t size) throw(std::bad_alloc)
{
    void * location;
    location = malloc(size);
    LOG.MEMORY("Just malloced using new %i at %p",size,location);
    return location;
}

void operator delete(void* needsFreed) throw()
{
    LOG.MEMORY("About to free using delete location : %p",needsFreed);
    free(needsFreed);
}

void operator delete[](void* needsFreed) throw()
{
    LOG.MEMORY("About to free using delete location : %p",needsFreed);
    free(needsFreed);
}
*/
