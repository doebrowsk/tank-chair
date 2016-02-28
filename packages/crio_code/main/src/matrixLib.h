/* 
 * File:   matrixLib.h
 * Author: ben
 *
 * Created on December 16, 2009, 11:15 AM
 */

#ifndef _MATRIXLIB_H
#define	_MATRIXLIB_H

typedef struct matrixCL{
    int rows;
    int columns;
    float * data;
}matrixDef;

/*
 *  The layout of the matrix is as follows
 *
 *  a1 a2 a3
 *  a4 a5 a6
 *  a7 a8 a9
 *
 */

typedef matrixDef* matrix;

matrix Matrix_Create(int row,int columns);
matrix Matrix_Transpose(matrix A, matrix out);
matrix Matrix_Copy(matrix A,matrix newMatrix);
matrix Matrix_Zero(matrix A);
matrix Matrix_Product(matrix A,matrix B,matrix out);
matrix Matrix_Add(matrix A,matrix B,matrix out);
matrix Matrix_Subtract(matrix A,matrix B,matrix out);
void  Matrix_SetValue(matrix A, int row , int column , float value);
matrix Matrix_MakeI(matrix A);
void Matrix_Print(matrix A);
matrix Matrix_Inverse(matrix A);
float Matrix_GetValue(matrix A, int row, int column);

#endif	/* _MATRIXLIB_H */
