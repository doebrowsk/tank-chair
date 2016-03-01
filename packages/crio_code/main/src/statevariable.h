#ifndef STATEVARIABLE_H
#define STATEVARIABLE_H

#include"matrixLibStack.h"

/*
  x
  y
  theta
  */

// Struct to hold sensor values in maxtrix
typedef struct {
	matrix State;
	matrix Variance;
} SensorVariable;

// Limits the theta reading
float limitTheta(float Theta);

#endif
