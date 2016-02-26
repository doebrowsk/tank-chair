#include "mainRunner.h"
#include <stdio.h>
#include <tasklib.h>

/**
 * loads the dll libraries and spawns a task for the MAINCRIO
 */
void MyStart(void) {
	printf("Loading Dynamic Library...\n");
	/* load the library */
      	#if NiFpga_Windows || NiFpga_Ets
       	MODULE_ID lib =  LoadLibraryA("MainCRIO.dll");
     	#elif NiFpga_VxWorks
       	MODULE_ID lib =  VxLoadLibraryFromPath("MainCRIO.out", 0);
      	#else
	  printf("Error dudes loading MainCRIO.out\n");
      	#endif
	const char* const name = "MainCRIO";
	void** const address;
	SYM_TYPE type;
        STATUS retVal = symFindByName(sysSymTbl,
       	                      (char*)name,
                              (char**)address,
                              &type);
	if (retVal == OK)
        {
		taskSpawn("MainCRIO", 100, VX_FP_TASK, 0x8000, (FUNCPTR) *address, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	}
	else {
		perror("Error loading MainCRIO function");
	}
	/* Freeing the library makes shit go all explody if the task hasn't
	 * exited */

	/*printf("Freeing the library...\n");
	VxFreeLibrary(lib, 0);
	printf("Library freed...\n");*/
}
