#ifndef LINALG_H
#define LINALG_H

#include <stdio.h>
#include <math.h>

void LinAlg_zeromat(float A[][3]); //Creates a LinAlg of zeroes
void LinAlg_zerovec(float x[3]); //Creates a vector of zeroes
void LinAlg_eye(float A[][3]); //Creates the identity LinAlg
void LinAlg_matvecmul(float A[][3], float x[3], float b[3]); //A*x=b
void LinAlg_matmatmul(float A[][3], float B[][3], float C[][3]); //A*B=C
void LinAlg_matmatadd(float A[][3], float B[][3], float C[][3]); //A+B=C
void LinAlg_matmatsub(float A[][3], float B[][3], float C[][3]); //A-B=C
void LinAlg_matscalmult(float A[][3], float k, float C[][3]); //k*A=C 
void LinAlg_vecscalmult(float x[3], float y[3], float k); // y = k*x
float LinAlg_vecdot(float a[3], float b[3]); //aTb
void LinAlg_mat2colvecs(float R[][3], float r1[3], float r2[3], float r3[3]); //R => [r1 r2 r3]
void LinAlg_colvecs2mat(float R[][3], float r1[3], float r2[3], float r3[3]); //[r1 r2 r3] => R
void LinAlg_matnormalize(float R[][3]); //Re-orthogonalizes and normalizes R
void LinAlg_vecvecadd(float a[3], float b[3], float c[3]); //c = a+b
void LinAlg_vecvecsub(float a[3], float b[3], float c[3]); //c = a-b
void LinAlg_vec2skew(float x[3], float S[][3]); //Skew(x) = S
void LinAlg_matcopy(float A[][3], float B[][3]); //B = A
void LinAlg_veccopy(float a[3], float b[3]); // b = a
void LinAlg_transpose(float A[][3], float AT[][3]); //AT = A^T
float LinAlg_vecnorm(float x[3]); //|x|
void LinAlg_normalize(float x[3], float x_norm[3]); //x_norm = x/|x|
int LinAlg_factorial(int k); //k!
void LinAlg_expm(float A[][3], float eA[][3], int k); // exp(A) = I + A + A^2/2 +...+ A^k/k!
float LinAlg_det(float A[][3]); //Determinant 
void LinAlg_rotX(float t, float R[][3]);
void LinAlg_rotY(float t, float R[][3]);
void LinAlg_rotZ(float t, float R[][3]);
void LinAlg_printscal(float a);
void LinAlg_printvec(float v[3]);
void LinAlg_printmat(float A[][3]);
void LinAlg_printvec_comma_separated(float v[3]);

#endif
