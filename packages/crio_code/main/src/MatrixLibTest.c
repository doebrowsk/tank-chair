/* 
 * File:   MatrixLibTest.c
 * Author: ben
 *
 * Created on January 11, 2010, 2:59 PM
 */

#include <stdio.h>
#include <stdlib.h>

#include <stdarg.h>
#include "matrixLibStack.h"

/*
 * 
 */

#define MSG_BUFFER_LENGTH 1024

char buffer[MSG_BUFFER_LENGTH];
char buffer2[MSG_BUFFER_LENGTH];

void _HarlieLogWrite(char *pType, char *pMsg)
{
    printf( "%s :\t%s \r\n",pType, pMsg);
}

void _LOG_INFO(char * msg, ...)
{
    va_list args;
    va_start (args, msg);
    vsprintf (buffer,msg, args);
    va_end (args);
    _HarlieLogWrite("INFO",buffer);
}
void _LOG_VERBOSE(char * msg, ...)
{
    va_list args;
    va_start (args, msg);
    vsprintf (buffer,msg, args);
    va_end (args);
    _HarlieLogWrite("VERBOSE",buffer);
}

void _LOG_DATA(char * msg, ...)
{
    va_list args;
    va_start (args, msg);
    vsprintf (buffer,msg, args);
    va_end (args);
    _HarlieLogWrite("DATA",buffer);
}

void _LOG_MATRIX(char * msg, ...)
{
    va_list args;
    va_start (args, msg);
    vsprintf (buffer,msg, args);
    va_end (args);
    _HarlieLogWrite("MATRIX",buffer);
}

void _LOG_KALMAN(char * msg, ...)
{
    va_list args;
    va_start (args, msg);
    vsprintf (buffer,msg, args);
    va_end (args);
    _HarlieLogWrite("KALMAN",buffer);
}


void _LOG_GPS(char * msg, ...)
{
    va_list args;
    va_start (args, msg);
    vsprintf (buffer,msg, args);
    va_end (args);
    _HarlieLogWrite("GPS",buffer);
}

void _LOG_YAW(char * msg, ...)
{
    va_list args;
    va_start (args, msg);
    vsprintf (buffer,msg, args);
    va_end (args);
    _HarlieLogWrite("YAW",buffer);
}

void _LOG_ODOMETRY(char * msg, ...)
{
    va_list args;
    va_start (args, msg);
    vsprintf (buffer,msg, args);
    va_end (args);
    _HarlieLogWrite("ODOMETRY ",buffer);
}


void _LOG_ERROR(char * msg, ...)
{
    va_list args;
    va_start (args, msg);
    vsprintf (buffer,msg, args);
    va_end (args);
    _HarlieLogWrite("ERROR ",buffer);
}



int main(int argc, char** argv) {
    /*--------------------------------------------------------------
     * Test 1
     */
    LOG.MATRIX = &_LOG_MATRIX;
    LOG.VERBOSE = &_LOG_VERBOSE;
    matrix A = Matrix_Create(2,2);
    matrix B = Matrix_Create(2,2);
    matrix C = Matrix_Create(2,2);

    printf("Test 1 \n");
    printf("Add a 2|2 of zeros to a 2|2 Identity and return\n");
    A = Matrix_MakeI(2);
    printf("The I\n");
    Matrix_Print(A);
    printf("The zero\n");
    Matrix_Print(B);
    C=Matrix_Add(A,B);
    printf("The result\n");
    Matrix_Print(C);

    /*--------------------------------------------------------------
     * Test 2
     */
    matrix E = Matrix_Create(3,2);
    matrix F = Matrix_Create(2,3);
    matrix G = Matrix_Create(3,3);

    printf("Test 2 \n");
    printf("Multiply 3|2 of zeros to a 2|3 and return\n");
    printf("The 3|2 , it should have the following values\n");
    printf("1  2\n");
    printf("3  4\n");
    printf("5  6\n");
    printf("The matrix \n");
    Matrix_Print(E);
    E = Matrix_SetValue(E,1,1,1);
    E = Matrix_SetValue(E,1,2,2);
    E = Matrix_SetValue(E,2,1,3);
    E = Matrix_SetValue(E,2,2,4);
    E = Matrix_SetValue(E,3,1,5);
    E = Matrix_SetValue(E,3,2,6);
    Matrix_Print(E);

    printf("The 3|2 , it should have the following values\n");
    printf("1  2  3\n");
    printf("4  5  6\n");
    printf("The matrix \n");
    F = Matrix_SetValue(F,1,1,1);
    F = Matrix_SetValue(F,1,2,2);
    F = Matrix_SetValue(F,1,3,3);
    F = Matrix_SetValue(F,2,1,4);
    F = Matrix_SetValue(F,2,2,5);
    F = Matrix_SetValue(F,2,3,6);
    Matrix_Print(F);
    G = Matrix_Product(E,F);
    printf("The result\n");
    Matrix_Print(G);

    printf("The solution should be\n");
    printf(" 9  12  15\n");
    printf("19  26  33\n");
    printf("29  40  51\n\n");

    printf("Now the 2|3 times the 3|2\nThe result\n");

    matrix H = Matrix_Create(2,2);
    H = Matrix_Product(F,E);
    Matrix_Print(H);
    
    printf("The solution should be\n");
    printf("22  28\n");
    printf("49  64\n\n");

    /*--------------------------------------------------------------
     * Test 3
     */
    printf("Test 3 \n");
    printf("Inverse of the 3|3\n");
    G = Matrix_Inverse(G);
    Matrix_Print(G);
    printf("The expected results as per QT octave\n");
    printf(" 6.4339e+14  -1.2868e+15   6.4339e+14\n");
    printf("-1.2868e+15   2.5736e+15  -1.2868e+15\n");
    printf(" 6.4339e+14  -1.2868e+15   6.4339e+14\n\n");

    printf("Inverse of the 2|2\nThe result\n");
    H = Matrix_Inverse(H);
    Matrix_Print(H);
    printf("The expected results as per QT octave\n");
    printf(" 1.77778  -0.77778\n");
    printf("-1.36111   0.61111\n\n");
    
    /*--------------------------------------------------------------
     * Test 4
     */

    printf("Test 4\n");
    printf("Test copy method.  This should output the same 2|2 array found in test 3\n");
    matrix I= Matrix_Create(2,2);
    I= Matrix_Copy(H);
    printf("The result\n");
    Matrix_Print(I);
    printf("The expected results\n");
    Matrix_Print(H);

    /*--------------------------------------------------------------
     * Test 5
     */

    printf("Test 5\n");
    printf("Test getting a value out of an matrix.  This should output the same 2|2 array found in test 3 in a list form\n");

    printf("The value list is as follows going 1|1 1|2 2|1 2|2\n");

    printf("%f\n",Matrix_GetValue(I,1,1));
    printf("%f\n",Matrix_GetValue(I,1,2));
    printf("%f\n",Matrix_GetValue(I,2,1));
    printf("%f\n",Matrix_GetValue(I,2,2));

    /*--------------------------------------------------------------
     * Test 6
     */

    printf("Test 6\n");
    printf("Transpose a matrix\n");

    matrix TP = Matrix_Create(4,1);
    matrix TPT = Matrix_Create(1,4);
    TP = Matrix_SetValue(TP,1,1,1);
    TP = Matrix_SetValue(TP,3,1,2);
    TP = Matrix_SetValue(TP,2,1,7);
    TPT = Matrix_Transpose(TP);
    Matrix_Print(TP);
    Matrix_Print(TPT);

    Matrix_Print(H);
    Matrix_Print(Matrix_Transpose(H));

    /*--------------------------------------------------------------
     * Test 7
     */

    printf("Test 7\n");
    printf("Subtraction\n");

    matrix X = Matrix_Create(4,1);
    matrix Z = Matrix_Create(4,1);
    matrix Y = Matrix_Create(4,1);

    X = Matrix_SetValue(X,1,1,1);

    X = Matrix_SetValue(X,3,1,12);
    X = Matrix_SetValue(X,2,1,3);

    Z = Matrix_SetValue(Z,1,1,1);
    Z = Matrix_SetValue(Z,3,1,1);
    Z = Matrix_SetValue(Z,2,1,1);

    Matrix_Print(Z);
    Matrix_Print(X);

    Y=Matrix_Subtract(X,Z);
    Matrix_Print(Y);

    /*--------------------------------------------------------------
     * Test 7
     */

    printf("Test 7\n");
    printf("Addition\n");

    Matrix_Print(Z);
    Matrix_Print(X);

    Y = Matrix_Add(X,Z);
    Matrix_Print(Y);

    return (EXIT_SUCCESS);
}
