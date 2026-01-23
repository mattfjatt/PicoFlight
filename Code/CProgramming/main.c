#include "stdio.h"
#include "stdlib.h"
#include "math.h"

void switch_rows_with_P_below_diag(int N, int diag_kk, int P[N], double A[N][N]);
void update_global_permutation_vector(int N, const int P_update[N], int P[N]);
void multiply_row(double A[][3], int r, double k);
void add_multiple_of_row_to_row(int N, int M, double A[N][M], int row_start_index, int from_row, int to_row, double k);
void permute_vector_with_P(int N, int P[N], double b[N]);
void printmat(int N, int M, const double A[N][M]);
void printvec(int N, double v[N]);
void printvec_int(int N, int v[N]);
void LU_decomposition(int N, int M, double A[N][M], double L[N][M], double U[N][M]);
int PLU_decomposition_NXN(int N, double A[N][N], double P[N][N], double L[N][N], double U[N][N]);
int PLU_decomposition_NXN_in_place(int N, int P[N], double LUMat[N][N]);
void kill_column_below_in_place(int N, int diag_kk, double LUMat[N][N]);
void find_permutation_matrix(int N, int diagonal_elem_kk, double M[N][N], double P[N][N]);
void find_permutation_vector(int N, int diag_kk, int P[N], double LUMat[N][N]);
void eye(int N,double A[N][N]);
void zeromat(int N, int M,double A[N][M]); //Convert to mxn DONE
void matcopy(int N, int M, const double A[N][M], double B[N][M]); //Convert to mxn DONE
void matmatmul_small(int N, int M, double A[N][M], double B[M][N], double C[N][N]); //Convert to mxn DONE
void matmatmul_no_alias(int N, int M, double A[N][M], double B[M][N], double C[N][N]);
void matmatmatmul_NXN(int N, double A[N][N], double B[N][N], double C[N][N], double D[N][N]); //Convert to mxn. IGNORE, too much shit to do and only used in square case anyway
void matmatsub(int N, int M, double A[N][M], double B[N][M], double C[N][M]); //Convert to mxn DONE
void matmatadd(int N, int M, double A[N][M], double B[N][M], double C[N][M]);
void init_matrices(int N, double A[N][N]); //Convert to mxn. IGNORE, this is just some shit
void kill_column_below(int N, int diag_kk, double U[N][N], double L[N][N]);
void set_3x3_mat(double a, double b, double c, 
                 double d, double e, double f,
                 double g, double h, double i, double A[3][3]);

void set_4x4_mat(double a, double b, double c, double d, 
                 double e, double f, double g, double h,
                 double i, double j, double k, double l,
                 double m, double n, double o, double p, double A[4][4]);

void print_PLU_decomp(int N, double A[N][N], double P[N][N], double L[N][N], double U[N][N], int rank);
void solve_lower_diagonal(int N, double L[N][N], double x[N], double b[N]);
void solve_upper_diagonal(int N, double U[N][N], double x[N], double b[N]);
void solve_linear_system_NXN(int N, double A[N][N], double x[N], double b[N]);
void solve_linear_system_NXN_in_place(int N, double A[N][N], double x[N], double b[N]);
void matvecmul(int N, int M, double A[N][M], double x[M], double b[N]); //Convert to mxn DONE
void zerovec(int N, double x[N]);
void veccopy(int N, double a[N], double b[N]);
void mat_transpose(int N, int M, double A[N][M], double AT[M][N]); //Convert to mxn DONE
void matscalmult(int N, int M, double A[N][M], double k, double C[N][M]);
void vecscalmult(int N, double x[N], double y[N], double k);
void test_updated_functions();


//Takes in 
// - Optimization variable count,9 for ellipsoid. 
// - An array containing the variables
// - A magnetometer sample si of 3 parameters
// - This should probably be given a struct to work with as the argument lists are getting long
double evaluate_ri(int param_count, double opt_params[param_count], double si[3]);
void evaluate_r_vec(int param_count, int sample_count, double opt_params[param_count], double samples[sample_count][3], double r_vec[sample_count]);
void evaluate_gradient_ri(int param_count, double opt_params[param_count], double si[3], double grad_ri[param_count]);
void evaluate_jacobian_r(int param_count, int sample_count, double opt_params[param_count], double samples[sample_count][3], double J[sample_count][param_count]);
void evaluate_gradient_r(int param_count, int sample_count, double g[param_count], double JT[param_count][sample_count], double r_vec[sample_count]); //gradient g = J^T*r
void LM_solver();

//Large variables for LM-solver
double J[10][9];
double JT[9][10];
double JTJ[9][9];


int main()
{
    // double A[3][3];
    // set_3x3_mat(1.0, 2.0, 3.0, 
    //             4.0, 6.0, 1.0,
    //             5.0, 10.0,3.0, A);
    // double b[3]; b[0] = 1.0; b[1] = 2.0; b[2] = 3.0;
    // double x[3];
    // solve_linear_system_NXN(3,A,x,b);
    // printvec(3,x);
    test_updated_functions();
    // int N = 4;
    // int P0[4] = {1,0,2,3};
    // int P_global[4] = {0,1,2,3};
    // update_global_permutation_vector(N,P0,P_global);
    // printvec_int(N,P_global);
    // update_global_permutation_vector(N,P0,P_global);
    // printvec_int(N,P_global);
    // LM_solver();
    printf("END OF PROGRAM\n");
    return 0;
}

void test_updated_functions()
{
    int N = 4;
    int M = 4;
    double A[N][M]; //VLA init, goes on the stack
    double A_transposed[M][N];
    double AAT[N][N];
    zeromat(N,M,A);
    zeromat(M,N,A_transposed);
    zeromat(N,N,AAT);
    set_4x4_mat(4,3,5,0,
                1,5,0,9,
                3,1,7,4,
                5,3,1,1,A);
    printf("A = \n");
    printmat(N,M,A);
    mat_transpose(N,M,A,A_transposed);
    printf("AT = \n");
    printmat(M,N,A_transposed);
    matmatmul_no_alias(N,M,A,A_transposed,AAT);
    printf("AAT = \n");
    printmat(N,N,AAT);

    printf("Add multiple of row to row testing\n");
    printmat(N,M,A);
    add_multiple_of_row_to_row(N,M,A,0,0,0,1.0);
    printmat(N,M,A);

    printf("Testing matscalmult function\n");
    double F[3][3];
    double O[3][3];
    eye(3,F);
    matscalmult(3,3,F,10,O);
    printmat(3,3,O);

    printf("Testing vecscal function\n");
    double s[3] = {1.0, 2.0, 3.0};
    vecscalmult(3,s,s,-0.1);
    printvec(3,s);

    //Testing in-place PLU
    double K[N][N];
    set_4x4_mat(4,7,5,0,
                1,5,0,9,
                3,1,7,4,
                5,6,1,1,K);
    int permutation_vector[N];
    PLU_decomposition_NXN_in_place(N,permutation_vector,K);
    // double P[N][N];
    // double L[N][N];
    // double U[N][N];
    // PLU_decomposition_NXN(N,K,P,L,U);
    printf("LU-matrix containing L and U:\n");
    printmat(N,N,K);

    

}

int PLU_decomposition_NXN_in_place(int N, int P[N], double LUMat[N][N])
{
    int P_local[N]; //Stack alloc, consider changing
    for(int i = 0; i < N; i++){
        P[i] = i; //Initialize the permutation vectors
        P_local[i] = i;
    }
    
    for(int diag_kk = 0; diag_kk < N - 1; diag_kk++){
        for(int i = 0; i < N; i++) P_local[i] = i; //Reset the local permutation vector
        find_permutation_vector(N, diag_kk, P_local, LUMat);
        switch_rows_with_P_below_diag(N, diag_kk, P_local, LUMat);
        kill_column_below_in_place(N, diag_kk, LUMat);
        printf("Iteration = %d\nLUMat current state is:\n", diag_kk);
        printmat(N,N,LUMat);
        update_global_permutation_vector(N, P_local, P);
    }
    //Add check the rank of U, do this by checking the fabs() of each pivot
    int rank = 0;
    for(int diag_kk = 0; diag_kk < N; diag_kk++){
        if(fabs(LUMat[diag_kk][diag_kk]) > 0.0){
            rank++;
        }
    }
    return rank;
}

void find_permutation_vector(int N, int diag_kk, int P[N], double LUMat[N][N])
{
    if(diag_kk > (N - 2)){
        printf("Error: Invalid index for finding largest pivot! Returning\n");
        return;
    }

    //P works in such a way that P = [0 1 2] corresponds to no row changes
    //P = [1 0 2] corresponds to row 1 being in 0th position and row 0 in 1st position

    //Find the largest element below and including diagonal element diag_kk
    double largest = 0.0;
    int largest_element_index = 0;
    for(int below_diag_kk = diag_kk; below_diag_kk < N; below_diag_kk++){
        double element = fabs(LUMat[below_diag_kk][diag_kk]);
        if(element > largest){
            largest = element;
            largest_element_index = below_diag_kk;
        }
    }

    if(largest_element_index > 0){//Update P. If largest element == 0, the diagonal element is the largest and no update will be done
        int tmp = P[diag_kk];
        P[diag_kk] = largest_element_index;
        P[largest_element_index] = tmp;
    }
}

void update_global_permutation_vector(int N, const int P_update[N], int P[N])
{
    //This function is only designed to accomododate one row exchange!
    //I.e P_update can not be equal to [1 0 3 2] as this corresponds to 2 row exchanges
    int index_mismatch = 0;
    const int max_switches = 2;
    for(int i = 0; i < N; i++){
        if(P_update[i] != i){
            index_mismatch++;
        }
        if(index_mismatch > max_switches){
            printf("Error: Invalid P_update passed to update_global_permutation_vector; it attempts more than one row exchange, aborting\n");
            return;
        }
    }

    for(int i = 0; i < N; i++){
        if(P_update[i] != i){ //If P_update[i] = i, no rows should change
            int tmp = P[i];
            P[i] = P[P_update[i]];
            P[P_update[i]] = tmp;
            break;
        }
    }
}

void LM_solver()
{
    int param_count = 9;
    int sample_count = 10;
    double opt_params[9] = {0.00038548, 0.00040191, 0.00039854, 34.9519,74.1484,-25.0253, 7.9526E-6, 1.0102E-5, 3.2523E-5};
    double r_vec[sample_count];

    double samples[10][3] =
    {
    {20.5625, 45.6875, -65.9375},
    {20.5625, 45.6875, -65.9375},
    {21.2500, 47.1250, -65.9375},
    {20.5625, 48.9375, -67.0625},
    {20.5625, 48.9375, -67.0625},
    {21.6250, 49.6250, -67.0625},
    {21.6250, 49.6250, -67.0625},
    {22.0000, 48.9375, -67.8750},
    {22.3125, 48.1875, -67.4375},
    {22.3125, 48.1875, -67.4375}
    };

    evaluate_r_vec(param_count,sample_count, opt_params,samples,r_vec);
    printvec(sample_count,r_vec);
    double si[3] = {1.0, 2.0, 3.0};
    double grad_ri[9];
    evaluate_gradient_ri(param_count,opt_params,si,grad_ri);
    printf("gradient ri = \n");
    printvec(param_count, grad_ri);
    evaluate_jacobian_r(param_count, 10, opt_params,samples,J);
    printf("J = \n");
    printmat(10,9,J);
    mat_transpose(10,9,J,JT);
    printf("JT = \n");
    printmat(9,10,JT);
    matmatmul_no_alias(9,10,JT,J,JTJ);
    printf("JTJ = \n");
    printmat(9,9,JTJ);
    printf("g = \n");
    double g[9];
    evaluate_gradient_r(9,10,g,JT,r_vec);
    printvec(9,g);

    //Solve for the A-matrix, A = JTJ + lambda*I
    double lambda = 1E-4; //This worked in matlab
    double lambdaEye[9][9];
    double A[9][9];
    eye(9,lambdaEye);
    matscalmult(9,9,lambdaEye,lambda,lambdaEye);
    printmat(9,9,lambdaEye);
    matmatadd(9,9,JTJ,lambdaEye,A);
    printf("JTJ + l*I = \n");
    printmat(9,9,A);
    //Want to solve (JTJ + l*I)deltax = - g
    //Switch sign of g
    vecscalmult(9,g,g,-1.0);
    double deltaX[9];
    solve_linear_system_NXN(9,A,deltaX,g);
    printf("Solution to (JTJ + l*I)deltax = - g is \n");
    printvec(9,deltaX); //Is correct!

    


}

void evaluate_gradient_r(int param_count, int sample_count, double g[param_count], double JT[param_count][sample_count], double r_vec[sample_count])
{
    //g = JT*r
    zerovec(param_count,g);
    for(int i = 0; i < param_count; i++){
        for(int j = 0; j < sample_count; j++){
            g[i] += JT[i][j]*r_vec[j];
        }
    }
}

void evaluate_gradient_ri(int param_count, double opt_params[param_count], double si[3], double grad_ri[param_count])
{
    double Ba = opt_params[0];
    double Bb = opt_params[1];
    double Bc = opt_params[2];
    double ui = (si[0] - opt_params[3]);
    double vi = (si[1] - opt_params[4]);
    double wi = (si[2] - opt_params[5]);
    double S0 = opt_params[6];
    double S1 = opt_params[7];
    double S2 = opt_params[8];
    
    grad_ri[0] = ui*ui;
    grad_ri[1] = vi*vi;
    grad_ri[2] = wi*wi;

    grad_ri[3] = -2*(Ba*ui + S0*vi + S1*wi);
    grad_ri[4] = -2*(Bb*vi + S0*ui + S2*wi);
    grad_ri[5] = -2*(Bc*wi + S1*ui + S2*vi);

    grad_ri[6] = 2*ui*vi; 
    grad_ri[7] = 2*ui*wi;
    grad_ri[8] = 2*vi*wi;

}

void evaluate_jacobian_r(int param_count, int sample_count, double opt_params[param_count], double samples[sample_count][3], double J[sample_count][param_count])
{
    //The jacobian of r_vec is a matrix where each row is a transposed gradient of ri
    double grad_ri[9];
    double si[3];
    for(int i = 0; i < sample_count; i++){
        si[0] = samples[i][0];
        si[1] = samples[i][1];
        si[2] = samples[i][2];
        evaluate_gradient_ri(param_count, opt_params, si, grad_ri);
        for(int j = 0; j < param_count; j++){
            J[i][j] = grad_ri[j];
        }
    }
}

void evaluate_r_vec(int param_count, int sample_count, double opt_params[param_count], double samples[sample_count][3], double r_vec[sample_count])
{
    double si[3];
    for(int i = 0; i < sample_count; i++){
        si[0] = samples[i][0];
        si[1] = samples[i][1];
        si[2] = samples[i][2];
        r_vec[i] = evaluate_ri(param_count, opt_params, si);
    }
}

double evaluate_ri(int param_count, double opt_params[param_count], double si[3])
{
    //opt_params = [Ba Bb Bc Px Py Px S0 S1 S2]
    double ri, ui, vi, wi;
    ui = (si[0] - opt_params[3]);
    vi = (si[1] - opt_params[4]);
    wi = (si[2] - opt_params[5]);
    ri = opt_params[0]*ui*ui +
         opt_params[1]*vi*vi +
         opt_params[2]*wi*wi + 
    2.0*(opt_params[6]*ui*vi + 
         opt_params[7]*ui*wi + 
         opt_params[8]*vi*wi) - 1.0;
    return ri;
}

void solve_linear_system_NXN_in_place(int N, double A[N][N], double x[N], double b[N])
{
    //P*L*U*x = b
    //L*U*x = transpose(P)*b = c
    //U*x = y
    //L*y = c solve for y, then solve for x
    int P[N];
    double c[N];
    double y[N];
    int rank = PLU_decomposition_NXN_in_place(N,P,A);
    if(rank < N){
        printf("System is rank-deficient!\n");
    }else{
        permute_vector_with_P();
        solve_lower_diagonal(N,A,y,c);
        solve_upper_diagonal(N,A,x,y);
    }
}

void solve_linear_system_NXN(int N, double A[N][N], double x[N], double b[N])
{
    //P*L*U*x = b
    //L*U*x = transpose(P)*b = c
    //U*x = y
    //L*y = c solve for y, then solve for x
    double P[N][N];
    double L[N][N];
    double U[N][N];
    double c[N];
    double y[N];
    int rank = PLU_decomposition_NXN(N,A,P,L,U);
    if(rank < N){
        printf("System is rank-deficient!\n");
    }else{
        mat_transpose(N,N,P,P);
        matvecmul(N,N,P,b,c);
        solve_lower_diagonal(N,L,y,c);
        solve_upper_diagonal(N,U,x,y);
    }
}

void permute_vector_with_P(int N, int P[N], double b[N])
{
    double tmp;
    for(int i = 0; i < N; i++){
        if(P[i] != i && i < N - 1){
            tmp = b[P[i]];
            b[P[i]] = b[i];
        }else if(i == N - 1){
            
        }
        
    }
}

void mat_transpose(int N, int M, double A[N][M], double AT[M][N])
{
    if(N == M){
        for(int i = 0; i < N; i++){
            for(int j = i; j < M; j++){
                double tmp = A[i][j]; 
                AT[i][j] = A[j][i];
                AT[j][i] = tmp;
            }
        }
    }else{
        if(A == AT){
            printf("Error: in-place non-square matrix transpose attempted, this is not possible, aborting\n");
            return;
        }else{
            //A and AT are separate variables and a tmp variable is not required
            for(int i = 0; i < N; i++){
                for(int j = 0; j < M; j++){
                    AT[j][i] = A[i][j];
                }
            }
        }
    }
}

void matvecmul(int N, int M, double A[N][M], double x[M], double b[N])
{
    if(N == M){
        double temp[N];
        zerovec(N,temp); //Found no cheaper way to do this than to create an entire temp vector
        for(int i = 0; i < N; i++){
            for(int j = 0; j < M; j++){
                temp[i] += A[i][j]*x[j];
            }
        }
        veccopy(N,temp,b);
    }else{
        if(x == b){
            printf("Error: Input and output vector can not be the same address in memory for N != M, aborting\n");
            return;
        }
        zerovec(N,b);
        for(int i = 0; i < N; i++){
            for(int j = 0; j < M; j++){
                b[i] += A[i][j]*x[j];
            }
        }
    }
}

void vecscalmult(int N, double x[N], double y[N], double k){
    for(int i = 0; i < N; i++){
        y[i] = k*x[i];
    }
}

void zerovec(int N, double x[N]){
    for(int i = 0; i < N; i++){
        x[i] = 0;
    }
}

void veccopy(int N, double a[N], double b[N]){
    for(int i = 0; i < N; i++){
        b[i] = a[i];
    }
}

void solve_upper_diagonal(int N, double U[N][N], double x[N], double b[N])
{
    //Solves for x in U*x=b
    for(int i = N - 1; i >= 0; i--){
        x[i] = b[i]/U[i][i];
        for(int j = i + 1; j <= N - 1; j++){
            x[i] = x[i] - U[i][j]*x[j]/U[i][i];
        }
    }
}

void solve_lower_diagonal(int N, double L[N][N], double x[N], double b[N])
{
    //Solves for x in L*x=b
    for(int i = 0; i < N; i++){
        x[i] = b[i]/L[i][i];
        for(int j = 0; j < i; j++){
            x[i] = x[i] - L[i][j]*x[j]/L[i][i];
        }
    }
}

void print_PLU_decomp(int N, double A[N][N], double P[N][N], double L[N][N], double U[N][N], int rank)
{
    printf("Matrices from PLU decomposition\nP = \n");
    printmat(N,N,P);
    printf("L = \n");
    printmat(N,N,L);
    printf("U = \n");
    printmat(N,N,U);
    printf("PLU = \n");
    double tmp[N][N];
    matmatmatmul_NXN(N,P,L,U,tmp);
    printmat(N,N,tmp);
    printf("A = \n");
    printmat(N,N,A);
    printf("A - PLU = \n");
    matmatsub(N,N,A,tmp,tmp);
    printmat(N, N, tmp);
    printf("rank(A) = rank(U) = %d\n", rank);
}

void set_3x3_mat(double a, double b, double c, 
                 double d, double e, double f,
                 double g, double h, double i, double A[3][3])
{
    A[0][0] = a; A[0][1] = b; A[0][2] = c;
    A[1][0] = d; A[1][1] = e; A[1][2] = f;
    A[2][0] = g; A[2][1] = h; A[2][2] = i;
}


void set_4x4_mat(double a, double b, double c, double d, 
                 double e, double f, double g, double h,
                 double i, double j, double k, double l,
                 double m, double n, double o, double p, double A[4][4])
{
    A[0][0] = a; A[0][1] = b; A[0][2] = c; A[0][3] = d;
    A[1][0] = e; A[1][1] = f; A[1][2] = g; A[1][3] = h;
    A[2][0] = i; A[2][1] = j; A[2][2] = k; A[2][3] = l;
    A[3][0] = m; A[3][1] = n; A[3][2] = o; A[3][3] = p;
}

void init_matrices(int N, double A[N][N])
{

    A[0][0] = 0.2;  A[0][1] = -1.5; A[0][2] = 4.7;  A[0][3] = 0.9;  A[0][4] = -2.1; A[0][5] = 5.4;  A[0][6] = 1.8;  A[0][7] = -3.6; A[0][8] = 2.0;
    A[1][0] = -0.7; A[1][1] = 6.1;  A[1][2] = 2.9;  A[1][3] = -4.3; A[1][4] = 1.5;  A[1][5] = -0.8; A[1][6] = 3.3;  A[1][7] = 2.7;  A[1][8] = -1.9;
    A[2][0] = 7.6;  A[2][1] = 1.2;  A[2][2] = -3.4; A[2][3] = 2.8;  A[2][4] = 6.0;  A[2][5] = -1.1; A[2][6] = -0.5; A[2][7] = 4.9;  A[2][8] = 3.1;

    A[3][0] = 2.4;  A[3][1] = -5.7; A[3][2] = 1.6;  A[3][3] = 4.2;  A[3][4] = -0.9; A[3][5] = 3.8;  A[3][6] = -2.6; A[3][7] = 1.1;  A[3][8] = 5.0;
    A[4][0] = -3.1; A[4][1] = 2.5;  A[4][2] = 6.8;  A[4][3] = -1.4; A[4][4] = 4.6;  A[4][5] = 0.7;  A[4][6] = 2.9;  A[4][7] = -5.2; A[4][8] = 1.3;
    A[5][0] = 4.9;  A[5][1] = 0.6;  A[5][2] = -2.8; A[5][3] = 5.5;  A[5][4] = 1.1;  A[5][5] = -4.0; A[5][6] = 6.3;  A[5][7] = 2.2;  A[5][8] = -0.7;

    A[6][0] = -1.8; A[6][1] = 3.4;  A[6][2] = 0.5;  A[6][3] = -2.9; A[6][4] = 5.7;  A[6][5] = 1.6;  A[6][6] = -4.4; A[6][7] = 6.0;  A[6][8] = 2.8;
    A[7][0] = -6.2;  A[7][1] = -0.4; A[7][2] = 3.1;  A[7][3] = 1.7;  A[7][4] = -5.3; A[7][5] = 2.6;  A[7][6] = 0.9;  A[7][7] = -1.2; A[7][8] = 4.5;
    A[8][0] = 1.0;  A[8][1] = 4.8;  A[8][2] = -1.6; A[8][3] = 6.4;  A[8][4] = 2.3;  A[8][5] = -3.7; A[8][6] = 5.1;  A[8][7] = 0.2;  A[8][8] = -2.5;
}

void find_permutation_matrix(int N, int diagonal_elem_kk, double M[N][N], double P[N][N])
{
    if(diagonal_elem_kk > (N - 2)){
        printf("Invalid index for finding largest pivot!\n");
        return;
    }
    eye(N,P);

    //We start looking for the largest pivot from diagonal element U[k][k] inclusive and down
    double largest = fabs(M[diagonal_elem_kk][diagonal_elem_kk]);
    int flip_with_this_row = diagonal_elem_kk;
    for(int k = diagonal_elem_kk; k < N; k++){
        double candidate = fabs(M[k][diagonal_elem_kk]);
        if(candidate > largest){
            largest = candidate;
            flip_with_this_row = k;
        }
    }
    //We change the matrix P such that when premultiplied with M, it changes the rows making the largest pivot end up at the diagonal
    //If the diagonal element is the largest, we change nothing.
    if(flip_with_this_row > diagonal_elem_kk){
        //Set relevant diagonal elements to 0
        P[diagonal_elem_kk][diagonal_elem_kk] = 0;
        P[flip_with_this_row][flip_with_this_row] = 0;
        //Then set relevant off-diagonal elements
        P[diagonal_elem_kk][flip_with_this_row] = 1;
        P[flip_with_this_row][diagonal_elem_kk] = 1;
    }
}

void matscalmult(int N, int M, double A[N][M], double k, double C[N][M])
{
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            C[i][j] = k*A[i][j];
        }
    }
}

int PLU_decomposition_NXN(int N, double A[N][N], double P[N][N], double L[N][N], double U[N][N])
{
    //This function finds:
    //P: The permutation matrix
    //L: Lower diagonal matrix
    //U: Upper diagonal matrix
    //Such that P*L*U = A. A property of P is inv(P) = P^T, thus L*U = P^T*A 

    //Initial conditions
    double P_iteration[N][N];
    matcopy(N,N,A,U); //U = A
    eye(N,L); //L = I
    eye(N,P); //P = I
    eye(N,P_iteration); //P_iteration = I
    
    //Iterating..
    for(int diag_kk = 0; diag_kk < N - 1; diag_kk++){
        eye(N, P_iteration); //Reset P_iteration
        find_permutation_matrix(N,diag_kk,U,P_iteration);
        matmatmatmul_NXN(N,P_iteration,L,P_iteration,L); //Update L:  L <-- P*L*P
        matmatmul_small(N,N,P_iteration,U,U); //Switch rows in U according to P: U <-- P*U
        kill_column_below(N, diag_kk,U,L);// Kill the column below the diagonal element diag_kk in U
        matmatmul_small(N,N,P,P_iteration,P); // Update the total permutation matrix: P < -- P*P_iteration
    }
    //Add check the rank of U, do this by checking the fabs() of each pivot
    int rank = 0;
    for(int diag_kk = 0; diag_kk < N; diag_kk++){
        if(fabs(U[diag_kk][diag_kk]) > 0.0){
            rank++;
        }
    }
    return rank;
}

void kill_column_below_in_place(int N, int diag_kk, double LUMat[N][N])
{
    for(int k = diag_kk + 1; k < N; k++){
        double LUMat_kk = LUMat[diag_kk][diag_kk];
        if(LUMat_kk != 0){ //<----- This should be slightly larger than 0; eps
            double factor = - LUMat[k][diag_kk]/LUMat_kk;
            add_multiple_of_row_to_row(N,N,LUMat,diag_kk,diag_kk,k,factor);
            //We need to add the negative of factor to L in order to keep the expression unchanged
            //Remember that for in-place, we store both L and U in the same matrix
            LUMat[k][diag_kk] = - factor;
        }
    }
}

void kill_column_below(int N, int diag_kk, double U[N][N], double L[N][N])
{
    for(int k = diag_kk + 1; k < N; k++){
        double U_kk = U[diag_kk][diag_kk];
        if(U_kk != 0){ //<----- This should be slightly larger than 0; eps
            double factor = - U[k][diag_kk]/U_kk;
            add_multiple_of_row_to_row(N,N,U,0,diag_kk,k,factor);
            //We need to add the negative of factor to L in order to keep the expression unchanged:
            L[k][diag_kk] = - factor;
        }
    }
}

void LU_decomposition(int N, int M, double A[N][M], double L[N][M], double U[N][M])
{
    matcopy(N,M,A,U); //Copy A into U, this is the starting point for U
    eye(N,L);//L = identity is the starting point for L, this can also be represented by a vector as only n*(n-1)/2 variables will be found 
    //For nxn matrices, only n*(n-1)/2 total row-ops are required to obtain L and U.

    //For 3x3 matrix:
    //printmat(U);
    //Step 1: Remove the leftmost element of the second row
    double fac = - U[1][0]/U[0][0];
    add_multiple_of_row_to_row(N,M,U,0,0,1,fac);
    //printmat(U);
    L[1][0] = - fac;
    //Step 2: Remove the leftmost element of the third row
    fac = - U[2][0]/U[0][0];
    add_multiple_of_row_to_row(N,M, U,0,0,2,fac);
    //printmat(U);
    L[2][0] = - fac;

    //Step 3: Remove the middle element of the third row
    fac = - U[2][1]/U[1][1];
    add_multiple_of_row_to_row(N,M, U,0,1,2,fac);
    L[2][1] = - fac;
    //printmat(U);
    //printmat(L);
    double res[3][3];
    zeromat(N, M, res);
    matmatmul_small(N, M,L,U,res);
    printmat(N, M,A);
    printmat(N, M,res);
}

void matmatmul_small(int N, int M, double A[N][M], double B[M][N], double C[N][N]){
    //A e NxM
    //B e MxN
    //C e NxN
    //Creating a temp matrix on the stack is a problem if N is very large, but even the Jacobian matrices of
    //J e 1000x9 and JT e 9x1000 will only yield JTJ e 9x9 when multiplied, this is 648 bytes, and the stack is 8kB 
    //But this is unnecessary as C will not be an alias anyway when N != M
    double temp[N][N];
    zeromat(N, N, temp); //temp set to be all zeroes
    for(int i = 0; i < N; i++){
        for(int j = 0; j < N; j++){
            for(int k = 0; k < M; k++){
                temp[i][j] += A[i][k]*B[k][j]; //Some inefficiency with indexing I need to consider here
            }
        }
    }
    matcopy(N, N, temp, C);
}

void matmatmul_no_alias(int N, int M, double A[N][M], double B[M][N], double C[N][N]){
    //A e NxM
    //B e MxN
    //C e NxN

    if(A == B || A == C || B == C){
        printf("Error: Aliasing not allowed in matmatmul_no_alias(...)!\n");
        return;
    }
    for(int i = 0; i < N; i++){
        for(int j = 0; j < N; j++){
            for(int k = 0; k < M; k++){
                C[i][j] += A[i][k]*B[k][j]; //Some inefficiency with indexing I need to consider here
            }
        }
    }
}

void matmatmatmul_NXN(int N, double A[N][N], double B[N][N], double C[N][N], double D[N][N])
{
    //D = A*B*C
    double tmp[N][N];
    matmatmul_small(N,N,A,B,tmp);
    matmatmul_small(N,N,tmp,C,D);
}

void matmatsub(int N, int M, double A[N][M], double B[N][M], double C[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            C[i][j] = A[i][j] - B[i][j];
        }
    }
}

void matmatadd(int N, int M, double A[N][M], double B[N][M], double C[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

void add_multiple_of_row_to_row(int N, int M, double A[N][M], int row_start_index, int from_row, int to_row, double k)
{
    if(from_row < 0 || from_row >= N || to_row < 0 || to_row >= N){
        printf("Error: from_rom/to_row outside of matrix dimenions in add_multiple_of_row_to_row, exiting\n");
        return;
    }

    for(int i = row_start_index; i < M; i++){
        A[to_row][i] += A[from_row][i]*k;
    }
}

void matcopy(int N, int M, const double A[N][M], double B[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            B[i][j] = A[i][j];
        }
    }
}

void zeromat(int N, int M,double A[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            A[i][j]= 0;
        }
    }
}

void eye(int N,double A[N][N])
{
    zeromat(N, N, A);
    for(int i = 0; i < N; i++){
        A[i][i]= 1;
    }
}

void multiply_row(double A[][3], int r, double k)
{
    for(int i = 0; i < 3; i++){
        A[r][i] = A[r][i]*k;
    }
}

void switch_rows_with_P_below_diag(int N, int diag_kk, int P[N], double A[N][N])
{
    int M = N;

    for(int i = diag_kk; i < N; i++){ //Only check/switch rows starting from the diagonal index
        if(P[i] != i){
            for(int j = 0; j < M; j++){
                double tmp = A[i][j];
                A[i][j] = A[P[i]][j];
                A[P[i]][j] = tmp;
            }
            break; //Exit the loop when a row swap has been performed
        }
    }
}

void printmat(int N, int M, const double A[N][M]){
    for(int i = 0; i < N; i++){
        printf("[");
        for(int j = 0; j < M; j++){
            if(j < 2){
                printf(" %.3f,",A[i][j]);
            }else{
               printf(" %.3f",A[i][j]); 
            }
        }
        printf("]\n");
    }
    printf("\n");
}

void printvec(int N, double v[N]){
    for(int i = 0; i < N; i++){
        printf("[%.5f]\n",v[i]);
    }
    printf("\n");
}

void printvec_int(int N, int v[N]){
    for(int i = 0; i < N; i++){
        printf("[%d]\n",v[i]);
    }
    printf("\n");
}
