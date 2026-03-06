#ifndef LINALG_H
#define LINALG_H

#include "headers/logging.h"
#include <math.h>

void linalg_zeromat(int n, int m,double A[n][m]); //Fills in a matrix of zeroes

void linalg_zerovec(int n, double x[n]); //Creates a vector of zeroes

void linalg_eye(int n,double A[n][n]); //Fills in the identity matrix

void linalg_matvecmul(int n, int m, double A[n][m], double x[m], double b[n]); //A*x=b

void linalg_matmatmul_small(int n, int m, double A[n][m], double B[m][n], double C[n][n]);  //A*B=C

void linalg_matmatadd(int n, int m, double A[n][m], double B[n][m], double C[n][m]); //A+B=C

void linalg_matmatsub(int n, int m, double A[n][m], double B[n][m], double C[n][m]); //A-B=C

void linalg_matscalmult(int n, int m, double A[n][m], double k, double C[n][m]); //k*A=C 

void linalg_vecscalmult(int n, double x[n], double y[n], double k); // y = k*x

double linalg_vecdot(int n, double a[n], double b[n]); //aTb

void linalg_mat2colvecs3x3(double R[][3], double r1[3], double r2[3], double r3[3]); //R => [r1 r2 r3]

void linalg_colvecs2mat3x3(double R[][3], double r1[3], double r2[3], double r3[3]); //[r1 r2 r3] => R

void linalg_matnormalizerotation(double R[][3]); //Re-orthogonalizes and normalizes R

void linalg_vecvecadd(int n, double a[n], double b[n], double c[n]); //c = a+b

void linalg_vecvecsub(int n, double a[n], double b[n], double c[n]); //c = a-b

void linalg_vec2skew3x3(double x[3], double S[][3]); //Skew(x) = S

void linalg_matcopy(int n, int m, const double A[n][m], double B[n][m]); //B = A

void linalg_veccopy(int n, double a[n], double b[n]); // b = a

void linalg_mattranspose(int n, int m, double A[n][m], double AT[m][n]); //AT = A^T

double linalg_vecnorm(int n, double x[n]); //|x|

void linalg_normalize(int n, double x[n], double x_norm[n]); //x_norm = x/|x|

int linalg_factorial(int k); //k!

void linalg_expm3x3(double A[][3], double eA[][3], int k); // exp(A) = I + A + A^2/2 +...+ A^k/k!

double linalg_det3x3(double A[][3]); //Determinant, upgrade this to work for NxN later?

void linalg_rotX(double t, double R[][3]);

void linalg_rotY(double t, double R[][3]);

void linalg_rotZ(double t, double R[][3]);

void linalg_printscal(double a);

void linalg_printvec(int n, double v[n]);

void linalg_printmat(int n, int m, const double A[n][m]);

void linalg_printvec_comma_separated(int n, double v[n]);

void linalg_printvec_int(int n, int v[n]);

void linalg_find_parallel_vec(int n, double a[n], double b[n], double b_parr[n]); //Finds the part of b, b_parr, that is parallel to a

void linalg_find_perpendicular_vec(int n, double a[n], double b[n], double b_perp[n]); //Finds the part of b, b_perp, that is perpendicular to a

double linalg_angle_between_vecs(int n, double a[n], double b[n], int return_degrees); //Returns result in degrees if returnDegrees > 0

//New functions required by the solver

int linalg_plu_decomposition_square_in_place(int n, int P[n], double LU_mat[n][n]);

void linalg_switch_rows_with_permutation_vector_below_diag(int n, int diag_kk, int P[n], double A[n][n]);

void linalg_update_global_permutation_vector(int n, const int P_update[n], int P[n]);

void linalg_add_multiple_of_row_to_row(int n, int m, double A[n][m], int row_start_index, int from_row, int to_row, double k);

void linalg_permute_vector_with_permutation_vector(int n, int P[n], double b[n]);

void linalg_kill_column_below_in_place(int n, int diag_kk, double LU_mat[n][n]);

void linalg_find_permutation_vector(int n, int diag_kk, int P[n], double LU_mat[n][n]);

void linalg_matmatmul_no_alias(int n, int m, double A[n][m], double B[m][n], double C[n][n]); //Used for larger matrices most likely defined globally

void linalg_solve_lower_diagonal(int n, double L[n][n], double x[n], double b[n]);

void linalg_solve_upper_diagonal(int n, double U[n][n], double x[n], double b[n]);

void linalg_solve_linear_system_square_in_place(int n, double A[n][n], double x[n], double b[n]); //This solver modifies b!!

#endif
