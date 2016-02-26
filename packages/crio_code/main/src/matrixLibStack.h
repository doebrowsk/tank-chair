/*
 * File:   matrixLib.h
 * Author: ben
 *
 * Created on December 16, 2009, 11:15 AM
 */
#ifndef _MATRIXLIB_H
#define	_MATRIXLIB_H

#ifdef MATRIXLIBTEST

typedef struct {
	void (*INFO)(char *, ...);
	/*So ERR is instead of ERROR. ERROR is some type of built in.*/
	void (*ERR)(char *, ...);
	void (*VERBOSE)(char *, ...);
	void (*MEMORY)(char *, ...);
	void (*DATA)(char *, ...);
	void (*MATRIX)(char *, ...);
	void (*KALMAN)(char *, ...);
	void (*YAW)(char *, ...);
	void (*GPS)(char *, ...);
	void (*ODOMETRY)(char *, ...);
}LogStruct;

LogStruct LOG;

#endif

#define MATRIXLIBDATATYPE double
#define MATLIB_EPS (1e-15)

#define maxSize 36

typedef struct matrixCL{
	int rows;
	int columns;
	MATRIXLIBDATATYPE data[maxSize];
}matrix;

/*
 *  The layout of the matrix is as follows
 *
 *  a1 a2 a3
 *  a4 a5 a6
 *  a7 a8 a9
 *
 */

matrix Matrix_Create(int row,int columns);
matrix Matrix_Transpose(matrix A);
matrix Matrix_Copy(matrix A);
matrix Matrix_Zero(matrix A);
matrix Matrix_Product(matrix A,matrix B);
matrix Matrix_Add(matrix A,matrix B);
matrix Matrix_Subtract(matrix A,matrix B);
matrix Matrix_SetValue(matrix A, int row , int column , MATRIXLIBDATATYPE value);
matrix Matrix_MakeI(int size);
void Matrix_Print(matrix A);
matrix Matrix_Inverse(matrix A);
MATRIXLIBDATATYPE Matrix_GetValue(matrix A, int row, int column);
MATRIXLIBDATATYPE Matrix_GetX(matrix A);
MATRIXLIBDATATYPE Matrix_GetY(matrix A);
MATRIXLIBDATATYPE Matrix_GetTheta(matrix A);

MATRIXLIBDATATYPE Matrix_GetXVar(matrix A);
MATRIXLIBDATATYPE Matrix_GetYVar(matrix A);
MATRIXLIBDATATYPE Matrix_GetThetaVar(matrix A);

MATRIXLIBDATATYPE Matrix_GetVel(matrix A);
MATRIXLIBDATATYPE Matrix_GetOmega(matrix A);

MATRIXLIBDATATYPE Matrix_GetVelVar(matrix A);
MATRIXLIBDATATYPE Matrix_GetOmegaVar(matrix A);
#endif	/* _MATRIXLIB_H */
