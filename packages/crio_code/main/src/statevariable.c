#include "statevariable.h"
#include <math.h>
#define TWOPI 6.28318531

// Limits the theta reading
float limitTheta(float Theta)
{
	Theta -= ((float)((int)(Theta/TWOPI)))*TWOPI;
	if ( Theta < 0 )
	{
		Theta += TWOPI;
	}
	return Theta;
}
