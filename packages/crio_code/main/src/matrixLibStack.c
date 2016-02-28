/*
 * File:   matrixLib.h
 * Author: ben
 *
 * Created on December 16, 2009, 11:15 AM
 */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include"matrixLibStack.h"
#ifndef MATRIXLIBTEST
#include "harlielog.h"
#else
#include <stdlib.h>
#include <stdio.h>
/*#define LOG.VERBOSE()
#define LOG.ERR()
#define LOG.MATRIX()
#define hMalloc malloc
*/
#endif

/*
 *  The layout of the matrix is as follows:
 *  a1 a2 a3
 *  a4 a5 a6
 *  a7 a8 a9
 */


/*
 *  Used for the printing method.
 */
char outStr[1024];

// Creates a matrix
matrix Matrix_Create(int row,int columns)
{
	matrix newMatrix;
	newMatrix.rows = row;
	newMatrix.columns = columns;
	/*This might not be nessessary*/
	newMatrix = Matrix_Zero(newMatrix);
	return newMatrix;
}

// Copies a matrix by coping the references of the values over.
matrix Matrix_Copy(matrix A)
{
	matrix newMatrix;
	newMatrix.columns = A.columns;
	newMatrix.rows = A.rows;
	int x;
	int y;
	for(x = 0;x < A.rows; x++)
	{
		for(y = 0; y<A.columns; y++)
		{
			newMatrix.data[A.columns*x + y]=A.data[A.columns*x + y] ;
		}
	}
	return newMatrix;
}

// Assigns zero values to the matrix
matrix Matrix_Zero(matrix A)
{
	int x;
	int y;
	for(x = 0; x < A.rows; x++)
	{
		for(y = 0; y < A.columns; y++)
		{
			A.data[A.columns*x + y]=0;
		}
	}
	/*
		   bzero(A.data,sizeof(MATRIXLIBDATATYPE)*A.rows*A.columns);
	   */
	return A;
}

// Transposoes a matrix 
matrix Matrix_Transpose(matrix A)
{
	matrix out;
	out.rows = A.columns;
	out.columns = A.rows;
	if (A.columns == out.rows && A.rows == out.columns)
	{
		int x;
		int y;
		for(x=0;x < A.rows;x++)
		{
			for(y=0;y<A.columns;y++)
			{
				out.data[y*out.columns+x]=A.data[x*A.columns+y];
			}
		}
	}
	else
	{
		LOG.ERR("Attempted to Transpose this matrix to this out matrix\n");
		Matrix_Print(A);
		Matrix_Print(out);
	}
	return out;
}

/*
 *  rows x columns product rows x columns  =   rows x columns
 *   A   x  B        X      C   x   D      =     E  X   F
 *   B = C
 *   E = A
 *   F = D
 *
 */
matrix Matrix_Product(matrix A,matrix B)
{
	matrix out;
	out.rows = A.rows;
	out.columns = B.columns;
	if (A.columns == B.rows && A.rows == out.rows && B.columns == out.columns)
	{
		int x;
		int y;
		for(x=0;x<A.rows;x++)
		{
			for(y=0;y<B.columns;y++)
			{
				MATRIXLIBDATATYPE sum = 0;
				int z = 0;
				for(z=0;z<A.columns;z++)
				{
					sum += A.data[x*A.columns + z]*B.data[z*B.columns + y];
				}
                                if(fabs(sum) < MATLIB_EPS) {
                                    sum = 0.0;
                                }
				out.data[x*B.columns+y] =sum;
			}
		}
	}
	else
	{
		LOG.ERR("Add failed columns and rows did not equal the expected values could not find the product");
	}
	return out;
}

// Adds matricies
matrix Matrix_Add(matrix A,matrix B)
{
	matrix out;
	out.columns = A.columns;
	out.rows = A.rows;
	if (A.rows == B.rows && A.columns == B.columns && out.columns == A.columns && out.rows == A.rows)
	{
		int x;
		int y;
		for(x=0;x<A.rows;x++)
		{
			for(y=0;y<A.columns;y++)
			{
				out.data[x*A.columns+y] = A.data[x*A.columns+y]+B.data[x*A.columns+y];
			}
		}
	}
	else
	{
		LOG.ERR("Add failed columns and rows did not equal the expected values matrix add failed");
	}
	return out;
}

// Subtracts matricies
matrix Matrix_Subtract(matrix A,matrix B)
{
	matrix out;
	out.rows = A.rows;
	out.columns = A.columns;
	if (A.rows == B.rows && A.columns == B.columns && out.columns == A.columns && out.rows == A.rows)
	{
		int x;
		int y;
		for(x=0;x<A.rows;x++)
		{
			for(y=0;y<A.columns;y++)
			{
				out.data[x*A.columns+y] = A.data[x*A.columns+y]-B.data[x*A.columns+y];
			}
		}
	}
	else
	{
		LOG.ERR("Subtract failed columns and rows did not equal the expected values matrix add failed");
	}
	return out;
}

/*
 *  Could be inlined in the future
 */
matrix  Matrix_SetValue(matrix A, int row , int column , MATRIXLIBDATATYPE value)
{
	row = row - 1;
	column = column - 1;
	if ( A.rows > row && row >= 0 && A.columns > column && column >=0)
	{
		A.data[row * A.columns + column] = value;
	}
	else
	{
		LOG.ERR("Could not set the value of the matrix because the row, columns were out of bounds");
	}
	return A;
}

// Returns the value in the matrix at row, column
MATRIXLIBDATATYPE Matrix_GetValue(matrix A, int row, int column)
{
	row = row - 1;
	column = column - 1;
	if ( A.rows > row && row >= 0 && A.columns > column && column >=0)
	{
		return A.data[row * A.columns + column];
	}
	else
	{
		LOG.ERR("Could not get the value of the matrix because the row, columns were out of bounds");
		return 0;
	}
}

// Makes a I matrix
matrix Matrix_MakeI(int size)
{
	int x;
	int y;
	matrix A;
	A.rows = size;
	A.columns = size;
	for(x=0;x<A.rows;x++)
	{
		for(y=0;y<A.columns;y++)
		{
			if(x == y)
			{
				A.data[y+x*A.columns] = 1;
			}
			else
			{
				A.data[y+x*A.columns] = 0;
			}
		}
	}
	return A;
}

/*
 *  Prints out the Matrix so that it can be easly read in the logs.
 */

void Matrix_Print(matrix A)
{
	int rows = A.rows;
	int columns = A.columns;
	int row = 0;
	outStr[0] = '\0';
	sprintf(outStr,"[\n");
	for(row = 0; row < rows;row++)
	{
		int column = 0;
		for(column =0; column < columns;column++)
		{
			sprintf(outStr,"%s%0- 4.4G\t\t",outStr,A.data[row * columns + column]);
		}
		sprintf(outStr,"%s ;\n",outStr);
	}
	LOG.MATRIX("row %i col %i  \n%s ]\n",rows,columns,outStr);
}

/*
 * This is a c implenetation of a template made my Mike Dinolfo in 1998.  Highly appriciated
 */
matrix Matrix_Inverse(matrix A)
{
	int actualsize = A.rows;
	int maxsize = A.columns;
	MATRIXLIBDATATYPE * data = &A.data[0];
	if (actualsize <= 0)
	{
		LOG.ERR("The size of the Matrix was less than 0");
		return A; /* sanity check*/
	}
	if (actualsize == 1)
	{
		/* if its a one by one then just inverse the value */
		matrix out = Matrix_Create(1,1);
		out = Matrix_SetValue(out,1,1,((MATRIXLIBDATATYPE)1.0)/(Matrix_GetValue(A,1,1)));
		return out; /* must be of dimension >= 2*/
	}
	int i;
	int j;
	int k;
	for (i = 1; i < actualsize; i++) data[i] /= data[0]; /* normalize row 0*/
	for (i = 1; i < actualsize; i++) {
		for (j = i; j < actualsize; j++) { /* do a column of L*/
			MATRIXLIBDATATYPE sum = 0.0;
			for (k = 0; k < i; k++)
				sum += data[j * maxsize + k] * data[k * maxsize + i];
			data[j * maxsize + i] -= sum;
		}
		if (i == actualsize - 1) continue;
		for (j = i + 1; j < actualsize; j++) { /* do a row of U*/
			MATRIXLIBDATATYPE sum = 0.0;
			for (k = 0; k < i; k++)
				sum += data[i * maxsize + k] * data[k * maxsize + j];
			data[i * maxsize + j] =
					(data[i * maxsize + j] - sum) / data[i * maxsize + i];
		}
	}
	for (i = 0; i < actualsize; i++) /* invert L*/
		for (j = i; j < actualsize; j++) {
		MATRIXLIBDATATYPE x = 1.0;
		if (i != j) {
			x = 0.0;
			for (k = i; k < j; k++)
				x -= data[j * maxsize + k] * data[k * maxsize + i];
		}
		data[j * maxsize + i] = x / data[j * maxsize + j];
	}
	for (i = 0; i < actualsize; i++) /* invert U*/
		for (j = i; j < actualsize; j++) {
		if (i == j) continue;
		MATRIXLIBDATATYPE sum = 0.0;
		for (k = i; k < j; k++)
			sum += data[k * maxsize + j]*((i == k) ? 1.0 : data[i * maxsize + k]);
		data[i * maxsize + j] = -sum;
	}
	for (i = 0; i < actualsize; i++) /* final inversion*/
		for (j = 0; j < actualsize; j++) {
		MATRIXLIBDATATYPE sum = 0.0;
		for (k = ((i > j) ? i : j); k < actualsize; k++)
			sum += ((j == k) ? 1.0 : data[j * maxsize + k]) * data[k * maxsize + i];
		data[j * maxsize + i] = sum;
	}
	return A;
}

MATRIXLIBDATATYPE Matrix_GetX(matrix A)
{
	return Matrix_GetValue(A,1,1);
}

MATRIXLIBDATATYPE Matrix_GetY(matrix A)
{
	return Matrix_GetValue(A,2,1);
}

MATRIXLIBDATATYPE Matrix_GetTheta(matrix A)
{
	return Matrix_GetValue(A,3,1);
}


MATRIXLIBDATATYPE Matrix_GetXVar(matrix A)
{
	return Matrix_GetValue(A,1,1);
}

MATRIXLIBDATATYPE Matrix_GetYVar(matrix A)
{
	return Matrix_GetValue(A,2,2);
}

MATRIXLIBDATATYPE Matrix_GetThetaVar(matrix A)
{
	return Matrix_GetValue(A,3,3);
}


MATRIXLIBDATATYPE Matrix_GetVel(matrix A)
{
	return Matrix_GetValue(A,4,1);
}
MATRIXLIBDATATYPE Matrix_GetOmega(matrix A)
{
	return Matrix_GetValue(A,5,1);
}

MATRIXLIBDATATYPE Matrix_GetVelVar(matrix A)
{
	return Matrix_GetValue(A,4,4);
}
MATRIXLIBDATATYPE Matrix_GetOmegaVar(matrix A)
{
	return Matrix_GetValue(A,5,5);
}
