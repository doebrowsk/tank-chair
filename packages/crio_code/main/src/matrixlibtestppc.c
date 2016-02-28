/*
 * File:   MatrixLibTest.c
 * Author: ben
 *
 * Created on January 11, 2010, 2:59 PM
 */

#include <vxWorks.h>
#include "matrixLibStack.h"
#include "matrixlibtestppc.h"
#include "harlielog.h"

STATUS runMatrixTests() {
    /*--------------------------------------------------------------
     * Test 1
     */
    matrix A = Matrix_Create(2,2);
    matrix B = Matrix_Create(2,2);
    matrix C = Matrix_Create(2,2);


    LOG.VERBOSE("Test 1");
    LOG.VERBOSE("Add a 2|2 of zeros to a 2|2 Identity and return");
    A = Matrix_MakeI(2);
    LOG.VERBOSE("The I");
    Matrix_Print(A);
    LOG.VERBOSE("The zero");
    Matrix_Print(B);
    C=Matrix_Add(A,B);
    LOG.VERBOSE("The result");
    Matrix_Print(C);

    /*--------------------------------------------------------------
     * Test 2
     */
    matrix E = Matrix_Create(3,2);
    matrix F = Matrix_Create(2,3);
    matrix G = Matrix_Create(3,3);

    LOG.VERBOSE("Test 2 \n");
    LOG.VERBOSE("Multiply 3|2 of zeros to a 2|3 and return\n");
    LOG.VERBOSE("The 3|2 , it should have the following values\n");
    LOG.VERBOSE("1  2\n");
    LOG.VERBOSE("3  4\n");
    LOG.VERBOSE("5  6\n");
    LOG.VERBOSE("The matrix \n");
    Matrix_Print(E);
    E = Matrix_SetValue(E,1,1,1);
    E = Matrix_SetValue(E,1,2,2);
    E = Matrix_SetValue(E,2,1,3);
    E = Matrix_SetValue(E,2,2,4);
    E = Matrix_SetValue(E,3,1,5);
    E = Matrix_SetValue(E,3,2,6);
    Matrix_Print(E);

    LOG.VERBOSE("The 3|2 , it should have the following values\n");
    LOG.VERBOSE("1  2  3\n");
    LOG.VERBOSE("4  5  6\n");
    LOG.VERBOSE("The matrix \n");
    F = Matrix_SetValue(F,1,1,1);
    F = Matrix_SetValue(F,1,2,2);
    F = Matrix_SetValue(F,1,3,3);
    F = Matrix_SetValue(F,2,1,4);
    F = Matrix_SetValue(F,2,2,5);
    F = Matrix_SetValue(F,2,3,6);
    Matrix_Print(F);
    G = Matrix_Product(E,F);
    LOG.VERBOSE("The result\n");
    Matrix_Print(G);

    LOG.VERBOSE("The solution should be\n");
    LOG.VERBOSE(" 9  12  15\n");
    LOG.VERBOSE("19  26  33\n");
    LOG.VERBOSE("29  40  51\n\n");

    LOG.VERBOSE("Now the 2|3 times the 3|2\nThe result\n");

    matrix H = Matrix_Create(2,2);
    H = Matrix_Product(F,E);
    Matrix_Print(H);

    LOG.VERBOSE("The solution should be\n");
    LOG.VERBOSE("22  28\n");
    LOG.VERBOSE("49  64\n\n");

    /*--------------------------------------------------------------
     * Test 3
     */
    LOG.VERBOSE("Test 3 \n");
    LOG.VERBOSE("Inverse of the 3|3\n");
    G = Matrix_Inverse(G);
    Matrix_Print(G);
    LOG.VERBOSE("The expected results as per QT octave\n");
    LOG.VERBOSE(" 6.4339e+14  -1.2868e+15   6.4339e+14\n");
    LOG.VERBOSE("-1.2868e+15   2.5736e+15  -1.2868e+15\n");
    LOG.VERBOSE(" 6.4339e+14  -1.2868e+15   6.4339e+14\n\n");

    LOG.VERBOSE("Inverse of the 2|2\nThe result\n");
    H = Matrix_Inverse(H);
    Matrix_Print(H);
    LOG.VERBOSE("The expected results as per QT octave\n");
    LOG.VERBOSE(" 1.77778  -0.77778\n");
    LOG.VERBOSE("-1.36111   0.61111\n\n");

    /*--------------------------------------------------------------
     * Test 4
     */

    LOG.VERBOSE("Test 4\n");
    LOG.VERBOSE("Test copy method.  This should output the same 2|2 array found in test 3\n");
    matrix I= Matrix_Create(2,2);
    I= Matrix_Copy(H);
    LOG.VERBOSE("The result\n");
    Matrix_Print(I);
    LOG.VERBOSE("The expected results\n");
    Matrix_Print(H);

    /*--------------------------------------------------------------
     * Test 5
     */

    LOG.VERBOSE("Test 5\n");
    LOG.VERBOSE("Test getting a value out of an matrix.  This should output the same 2|2 array found in test 3 in a list form\n");

    LOG.VERBOSE("The value list is as follows going 1|1 1|2 2|1 2|2\n");

    LOG.VERBOSE("%f\n",Matrix_GetValue(I,1,1));
    LOG.VERBOSE("%f\n",Matrix_GetValue(I,1,2));
    LOG.VERBOSE("%f\n",Matrix_GetValue(I,2,1));
    LOG.VERBOSE("%f\n",Matrix_GetValue(I,2,2));

    /*--------------------------------------------------------------
     * Test 6
     */

    LOG.VERBOSE("Test 6\n");
    LOG.VERBOSE("Transpose a matrix\n");

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

    LOG.VERBOSE("Test 7\n");
    LOG.VERBOSE("Subtraction\n");

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

    LOG.VERBOSE("Test 7\n");
    LOG.VERBOSE("Addition\n");

    Matrix_Print(Z);
    Matrix_Print(X);

    Y = Matrix_Add(X,Z);
    Matrix_Print(Y);

    LOG.VERBOSE("Test 8");
    LOG.VERBOSE("A number of operations that should result in a valid covariance matrix... ie symmetric");
    matrix i8 = Matrix_MakeI(5);
    matrix k8 = Matrix_Create(5,2);
    matrix h8 = Matrix_Create(2,5);
    matrix p8 = Matrix_Create(5,5);

    // Setup the initial covariance matrix
    p8 = Matrix_SetValue(p8, 1, 1, 1e4);
    p8 = Matrix_SetValue(p8, 1, 4, 0.2);
    p8 = Matrix_SetValue(p8, 2, 2, 1e4);
    p8 = Matrix_SetValue(p8, 3, 3, 3.004);
    p8 = Matrix_SetValue(p8, 3, 5, 0.2);
    p8 = Matrix_SetValue(p8, 4, 1, 0.2);
    p8 = Matrix_SetValue(p8, 4, 4, 20);
    p8 = Matrix_SetValue(p8, 5, 3, 0.2);
    p8 = Matrix_SetValue(p8, 5, 5, 20);

    // Setup the kalman gain 
    k8 = Matrix_SetValue(k8, 1, 1, 0.25);
    k8 = Matrix_SetValue(k8, 1, 2, 0.25);
    k8 = Matrix_SetValue(k8, 3, 1, 0.5);
    k8 = Matrix_SetValue(k8, 3, 2, -0.5);
    k8 = Matrix_SetValue(k8, 4, 1, 25.);
    k8 = Matrix_SetValue(k8, 4, 2, 25.);
    k8 = Matrix_SetValue(k8, 5, 1, 50.);
    k8 = Matrix_SetValue(k8, 5, 2, -50.);

    // Setup the H matrix 
    h8 = Matrix_SetValue(h8, 1, 4, 0.02);
    h8 = Matrix_SetValue(h8, 1, 5, 0.01);
    h8 = Matrix_SetValue(h8, 2, 4, 0.02);
    h8 = Matrix_SetValue(h8, 2, 5, -0.01);

    LOG.VERBOSE("K MATRIX");
    Matrix_Print(k8);
    LOG.VERBOSE("H MATRIX");
    Matrix_Print(h8);

    // Do the covariance correction step P = (I - K*H)*P 
    matrix temp1 = Matrix_Product(k8,h8);
    LOG.VERBOSE("K*H");
    Matrix_Print(temp1);
    matrix temp2 = Matrix_Subtract(i8, temp1);
    LOG.VERBOSE("I - temp1");
    Matrix_Print(temp2);
    matrix temp3 = Matrix_Product(temp2, p8);
    LOG.VERBOSE("The output covariance matrix is");
    Matrix_Print(temp3);

    return (OK);
}
