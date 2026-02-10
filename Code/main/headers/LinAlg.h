#ifndef LINALG_H
#define LINALG_H

#include "headers/Logging.h"
#include <math.h>

void LinAlg_zeromat(int N, int M,double A[N][M]); //Fills in a matrix of zeroes

void LinAlg_zerovec(int N, double x[N]); //Creates a vector of zeroes

void LinAlg_eye(int N,double A[N][N]); //Fills in the identity matrix

void LinAlg_matvecmul(int N, int M, double A[N][M], double x[M], double b[N]); //A*x=b

void LinAlg_matmatmul_small(int N, int M, double A[N][M], double B[M][N], double C[N][N]);  //A*B=C

void LinAlg_matmatadd(int N, int M, double A[N][M], double B[N][M], double C[N][M]); //A+B=C

void LinAlg_matmatsub(int N, int M, double A[N][M], double B[N][M], double C[N][M]); //A-B=C

void LinAlg_matscalmult(int N, int M, double A[N][M], double k, double C[N][M]); //k*A=C 

void LinAlg_vecscalmult(int N, double x[N], double y[N], double k); // y = k*x

double LinAlg_vecdot(int N, double a[N], double b[N]); //aTb

void LinAlg_mat2colvecs3x3(double R[][3], double r1[3], double r2[3], double r3[3]); //R => [r1 r2 r3]

void LinAlg_colvecs2mat3x3(double R[][3], double r1[3], double r2[3], double r3[3]); //[r1 r2 r3] => R

void LinAlg_matnormalizerotation(double R[][3]); //Re-orthogonalizes and normalizes R

void LinAlg_vecvecadd(int N, double a[N], double b[N], double c[N]); //c = a+b

void LinAlg_vecvecsub(int N, double a[N], double b[N], double c[N]); //c = a-b

void LinAlg_vec2skew3x3(double x[3], double S[][3]); //Skew(x) = S

void LinAlg_matcopy(int N, int M, const double A[N][M], double B[N][M]); //B = A

void LinAlg_veccopy(int N, double a[N], double b[N]); // b = a

void LinAlg_mattranspose(int N, int M, double A[N][M], double AT[M][N]); //AT = A^T

double LinAlg_vecnorm(int N, double x[N]); //|x|

void LinAlg_normalize(int N, double x[N], double x_norm[N]); //x_norm = x/|x|

int LinAlg_factorial(int k); //k!

void LinAlg_expm3x3(double A[][3], double eA[][3], int k); // exp(A) = I + A + A^2/2 +...+ A^k/k!

double LinAlg_det3x3(double A[][3]); //Determinant, upgrade this to work for NxN later?

void LinAlg_rotX(double t, double R[][3]);

void LinAlg_rotY(double t, double R[][3]);

void LinAlg_rotZ(double t, double R[][3]);

void LinAlg_printscal(double a);

void LinAlg_printvec(int N, double v[N]);

void LinAlg_printmat(int N, int M, const double A[N][M]);

void LinAlg_printvec_comma_separated(int N, double v[N]);

void LinAlg_printvec_int(int N, int v[N]);

void LinAlg_find_parallel_vec(int N, double a[N], double b[N], double b_parr[N]); //Finds the part of b, b_parr, that is parallel to a

void LinAlg_find_perpendicular_vec(int N, double a[N], double b[N], double b_perp[N]); //Finds the part of b, b_perp, that is perpendicular to a

//New functions required by the solver

int LinAlg_PLU_decomposition_NXN_in_place(int N, int P[N], double LUMat[N][N]);

void LinAlg_switch_rows_with_P_below_diag(int N, int diag_kk, int P[N], double A[N][N]);

void LinAlg_update_global_permutation_vector(int N, const int P_update[N], int P[N]);

void LinAlg_add_multiple_of_row_to_row(int N, int M, double A[N][M], int row_start_index, int from_row, int to_row, double k);

void LinAlg_permute_vector_with_P(int N, int P[N], double b[N]);

void LinAlg_kill_column_below_in_place(int N, int diag_kk, double LUMat[N][N]);

void LinAlg_find_permutation_vector(int N, int diag_kk, int P[N], double LUMat[N][N]);

void LinAlg_matmatmul_no_alias(int N, int M, double A[N][M], double B[M][N], double C[N][N]); //Used for larger matrices most likely defined globally

void LinAlg_solve_lower_diagonal(int N, double L[N][N], double x[N], double b[N]);

void LinAlg_solve_upper_diagonal(int N, double U[N][N], double x[N], double b[N]);

void LinAlg_solve_linear_system_NXN_in_place(int N, double A[N][N], double x[N], double b[N]); //This solver modifies b!!

#endif
