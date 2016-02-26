/*
 * File:   matrixLib.h
 * Author: ben
 *
 * Created on December 16, 2009, 11:15 AM
 */

#include <math.h>
#include <string.h>
#include <stdio.h>
#include"matrixLib.h"
#ifndef MATRIXLIBTEST
#include "harlielog.h"
#include "harliealloc.h"
#else
#include <stdlib.h>
#include <stdio.h>
#define LOG.VERBOSE printf
#define LOG.ERR printf
#define hMalloc malloc
#endif

/*
 *  The layout of the matrix is as follows
 *
 *  a1 a2 a3
 *  a4 a5 a6
 *  a7 a8 a9
 *
 */

/*
 *  Used for the printing method.
 */
char outStr[1024];

matrix Matrix_Create(int row,int columns)
{
	matrix newMatrix = hMalloc(sizeof(matrixDef));
	newMatrix->data = hMalloc(sizeof(float) * row * columns);
	newMatrix->rows = row;
	newMatrix->columns = columns;
        /*This might not be nessessary*/
        Matrix_Zero(newMatrix);
	return newMatrix;
}

// Coppies a matrix into another matrix
matrix Matrix_Copy(matrix A,matrix newMatrix)
{
    if(A->rows == newMatrix->rows && A->columns == newMatrix->columns)
	{
		int x;
		int y;
		for(x=0;x < A->rows;x++)
		{
			for(y=0;y<A->columns;y++)
			{
                newMatrix->data[A->columns*x + y]=A->data[A->columns*x + y];
			}
		}
	}
    return newMatrix;
}

// Maxkes a zero matrix
matrix Matrix_Zero(matrix A)
{
	int x;
	int y;
	for( x = 0;x < A->rows; x++)
	{
		for(y=0;y<A->columns;y++)
		{
			A->data[A->columns + x]=0; 
		}
	}
	/*
	   bzero(A->data,sizeof(float)*A->rows*A->columns);
	   */
    return A;
}

// Transposes a matrix
matrix Matrix_Transpose(matrix A, matrix out)
{
	if (A->columns == out->rows && A->rows == out->columns)
	{
		int x;
		int y;
        for(x=0;x < A->rows;x++)
		{
            for(y=0;y<A->columns;y++)
			{
                out->data[y*out->columns+x]=A->data[x*A->columns+y];
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
matrix Matrix_Product(matrix A,matrix B,matrix out)
{
	if (A->columns == B->rows && A->rows == out->rows && B->columns == out->columns)
	{
		int x;
		int y;
		for(x = 0; x < A->rows; x++)
		{
			for(y = 0; y < B->columns; y++)
			{
				float sum = 0;
				int z = 0;
				for(z = 0; z < A->columns; z++)
				{
					sum += A->data[x*A->columns + z]*B->data[z*B->columns + y];
				}
				out->data[x*B->columns+y] =sum;
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
matrix Matrix_Add(matrix A,matrix B,matrix out)
{
	if (A->rows == B->rows && A->columns == B->columns && out->columns == A->columns && out->rows == A->rows)
	{
		int x;
		int y;
		for(x = 0; x < A->rows; x++)
		{
			for(y = 0; y < A->columns; y++)
			{
				out->data[x*A->columns+y] = A->data[x*A->columns+y]+B->data[x*A->columns+y];
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
matrix Matrix_Subtract(matrix A,matrix B,matrix out)
{
	if (A->rows == B->rows && A->columns == B->columns && out->columns == A->columns && out->rows == A->rows)
	{
		int x;
		int y;
		for(x = 0; x < A->rows; x++)
		{
			for(y = 0; y < A->columns; y++)
			{
				out->data[x*A->columns+y] = A->data[x*A->columns+y]-B->data[x*A->columns+y];
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
void  Matrix_SetValue(matrix A, int row , int column , float value)
{
	row = row - 1;
	column = column - 1;
	if ( A->rows > row && row >= 0 && A->columns > column && column >=0)
	{
		A->data[row * A->columns + column] = value;
	}
	else
	{
        LOG.ERR("Could not set the value of the matrix because the row, columns were out of bounds");
	}
}

// Returns the value at row, column in the matrix
float Matrix_GetValue(matrix A, int row, int column)
{
	row = row - 1;
	column = column - 1;
	if ( A->rows > row && row >= 0 && A->columns > column && column >=0)
	{
		return A->data[row * A->columns + column];
	}
	else
	{
        LOG.ERR("Could not get the value of the matrix because the row, columns were out of bounds");
		return 0;
	}
}

// Constructs an inverse matrix?
matrix Matrix_MakeI(matrix A)
{
	int x;
	int y;

	if ( A->rows !=A->columns)
	{
        LOG.ERR("The matrix was not a square, therefore an I matrix could not be made");
	}
	for(x=0;x<A->rows;x++)
	{
		for(y=0;y<A->columns;y++)
		{
			if(x == y)
			{
				A->data[y+x*A->columns] = 1;
			}
			else
			{
				A->data[y+x*A->columns] = 0;
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
	int rows = A->rows;
	int columns = A->columns;
	int row = 0;
	outStr[0] = '\0';
        sprintf(outStr,"[\n");
	for(row = 0; row < rows;row++)
	{
		int column = 0;
		for(column =0; column < columns;column++)
		{
			sprintf(outStr,"%s%0- 4.4G\t\t",outStr,A->data[row * columns + column]);
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
	int actualsize = A->rows;
	int maxsize = A->columns;
	float * data = A->data;
	if (actualsize <= 0)
	{
        LOG.ERR("The size of the Matrix was less than 0");
		return A; /* sanity check*/
	}
	if (actualsize == 1) 
	{
        LOG.ERR("The size must be greater than 1");
		return A; /* must be of dimension >= 2*/
	}
	int i;
	int j;
	int k;
	for (i = 1; i < actualsize; i++) data[i] /= data[0]; /* normalize row 0*/
	for (i = 1; i < actualsize; i++) {
		for (j = i; j < actualsize; j++) { /* do a column of L*/
			float sum = 0.0;
			for (k = 0; k < i; k++)
				sum += data[j * maxsize + k] * data[k * maxsize + i];
			data[j * maxsize + i] -= sum;
		}
		if (i == actualsize - 1) continue;
		for (j = i + 1; j < actualsize; j++) { /* do a row of U*/
			float sum = 0.0;
			for (k = 0; k < i; k++)
				sum += data[i * maxsize + k] * data[k * maxsize + j];
			data[i * maxsize + j] =
				(data[i * maxsize + j] - sum) / data[i * maxsize + i];
		}
	}
	for (i = 0; i < actualsize; i++) /* invert L*/
		for (j = i; j < actualsize; j++) {
			float x = 1.0;
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
			float sum = 0.0;
			for (k = i; k < j; k++)
				sum += data[k * maxsize + j]*((i == k) ? 1.0 : data[i * maxsize + k]);
			data[i * maxsize + j] = -sum;
		}
	for (i = 0; i < actualsize; i++) /* final inversion*/
		for (j = 0; j < actualsize; j++) {
			float sum = 0.0;
			for (k = ((i > j) ? i : j); k < actualsize; k++)
				sum += ((j == k) ? 1.0 : data[j * maxsize + k]) * data[k * maxsize + i];
			data[j * maxsize + i] = sum;
		}
	return A;
}
