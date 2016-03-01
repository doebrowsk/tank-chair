#include "microprocessorinterface.h"
#include "fpga.h"

const MicroprocessorInterface * MicroprocessorInit()
{
	return FPGA_Interface();
}

