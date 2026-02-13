#include "headers/LinAlg.h"

void LinAlg_zeromat(int N, int M,double A[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            A[i][j]= 0;
        }
    }
}

void LinAlg_zerovec(int N, double x[N]){
    for(int i = 0; i < N; i++){
        x[i] = 0;
    }
}

void LinAlg_eye(int N,double A[N][N])
{
    LinAlg_zeromat(N, N, A);
    for(int i = 0; i < N; i++){
        A[i][i]= 1;
    }
}

void LinAlg_matvecmul(int N, int M, double A[N][M], double x[M], double b[N])
{
    if(N == M){
        double temp[N];
        LinAlg_zerovec(N,temp); //Found no cheaper way to do this than to create an entire temp vector
        for(int i = 0; i < N; i++){
            for(int j = 0; j < M; j++){
                temp[i] += A[i][j]*x[j];
            }
        }
        LinAlg_veccopy(N,temp,b);
    }else{
        if(x == b){
            LOG("Error: Input and output vector can not be the same address in memory for N != M, aborting\n");
            return;
        }
        LinAlg_zerovec(N,b);
        for(int i = 0; i < N; i++){
            for(int j = 0; j < M; j++){
                b[i] += A[i][j]*x[j];
            }
        }
    }
}

void LinAlg_matmatmul_small(int N, int M, double A[N][M], double B[M][N], double C[N][N]){
    //A e NxM
    //B e MxN
    //C e NxN
    //Creating a temp matrix on the stack is a problem if N is very large, but even the Jacobian matrices of
    //J e 1000x9 and JT e 9x1000 will only yield JTJ e 9x9 when multiplied, this is 648 bytes, and the stack is 8kB 
    //But this is unnecessary as C will not be an alias anyway when N != M
    double temp[N][N];
    LinAlg_zeromat(N, N, temp); //temp set to be all zeroes
    for(int i = 0; i < N; i++){
        for(int j = 0; j < N; j++){
            for(int k = 0; k < M; k++){
                temp[i][j] += A[i][k]*B[k][j]; //Some inefficiency with indexing I need to consider here
            }
        }
    }
    LinAlg_matcopy(N, N, temp, C);
}

void LinAlg_vecvecadd(int N, double a[N], double b[N], double c[N]){
    for(int i = 0; i < N; i++){
        c[i] = a[i] + b[i];
    }
}

void LinAlg_vecvecsub(int N, double a[N], double b[N], double c[N]){
    for(int i = 0; i < N; i++){
        c[i] = a[i] - b[i];
    }
}

void LinAlg_vecscalmult(int N, double x[N], double y[N], double k){
    for(int i = 0; i < N; i++){
        y[i] = k*x[i];
    }
}

double LinAlg_vecdot(int N, double a[N], double b[N]){
    double c = 0;
    for(int i = 0; i < N; i++){
        c += a[i]*b[i];
    }
    return c;
}

void LinAlg_matmatadd(int N, int M, double A[N][M], double B[N][M], double C[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

void LinAlg_matmatsub(int N, int M, double A[N][M], double B[N][M], double C[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            C[i][j] = A[i][j] - B[i][j];
        }
    }
}

void LinAlg_matscalmult(int N, int M, double A[N][M], double k, double C[N][M])
{
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            C[i][j] = k*A[i][j];
        }
    }
}

void LinAlg_mat2colvecs3x3(double R[][3], double r1[3], double r2[3], double r3[3]){
    for(int i = 0; i < 3; i++){
        r1[i] = R[i][0];
        r2[i] = R[i][1];
        r3[i] = R[i][2];
    }
}

void LinAlg_colvecs2mat3x3(double R[][3], double r1[3], double r2[3], double r3[3]){
    for(int i = 0; i < 3; i++){
        R[i][0] = r1[i];
        R[i][1] = r2[i];
        R[i][2] = r3[i];
    }
}

void LinAlg_matnormalizerotation(double R[][3]){
    //Crude method of ensuring R remains in SO(3)
    int N = 3;
    double r1[3], r2[3], r3[3];
    double u1[3], u2[3], u3[3];
    double k1, k2, k3;
    double k1u1[3], k2u1[3], k3u2[3];
    LinAlg_mat2colvecs3x3(R, r1, r2, r3);
    //u1 = r1;
    LinAlg_veccopy(N,r1,u1);

    //u2 = r2 - k1*u1
    //k1 = (r2*u1)/(u1*u1)
    k1 = LinAlg_vecdot(N,r2,u1)/LinAlg_vecdot(N,u1,u1);
    LinAlg_vecscalmult(N,u1,k1u1, k1);
    LinAlg_vecvecsub(N,r2,k1u1,u2);

    //u3 = r3 - (r3*u1)/(u1*u1)*u1 - (r3*u2)/(u2*u2)*u2
    //u3 = r3 - k2*u1 - k3*u2;
    //k2 = (r3*u1)/(u1*u1)
    //k3 = (r3*u2)/(u2*u2)
    k2 = LinAlg_vecdot(N,r3,u1)/LinAlg_vecdot(N,u1,u1);
    k3 = LinAlg_vecdot(N,r3,u2)/LinAlg_vecdot(N,u2,u2);
    LinAlg_vecscalmult(N,u1,k2u1, k2);
    LinAlg_vecscalmult(N,u2,k3u2, k3);
    LinAlg_veccopy(N,r3,u3);
    LinAlg_vecvecsub(N,u3,k2u1,u3);
    LinAlg_vecvecsub(N,u3,k3u2,u3);

    //Normalize the orthogonal vectors 
    LinAlg_normalize(N,u1,u1);
    LinAlg_normalize(N,u2,u2);
    LinAlg_normalize(N,u3,u3);
    
    LinAlg_colvecs2mat3x3(R,u1,u2,u3);

}

void LinAlg_vec2skew3x3(double x[3], double S[][3]){
    S[0][0] = 0;
    S[1][1] = 0;
    S[2][2] = 0;

    S[0][1] = -x[2];
    S[0][2] =  x[1];
    S[1][2] = -x[0];
    S[1][0] =  x[2];
    S[2][0] = -x[1];
    S[2][1] =  x[0];
}

void LinAlg_matcopy(int N, int M, const double A[N][M], double B[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            B[i][j] = A[i][j];
        }
    }
}

void LinAlg_veccopy(int N, double a[N], double b[N]){
    for(int i = 0; i < N; i++){
        b[i] = a[i];
    }
}

void LinAlg_mattranspose(int N, int M, double A[N][M], double AT[M][N])
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
            LOG("Error: in-place non-square matrix transpose attempted, this is not possible, aborting\n");
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

double LinAlg_vecnorm(int N, double x[N]){
    double norm = 0;
    for(int i = 0; i < N; i++){
        norm += x[i]*x[i];
    }
    return sqrt(norm);
}

void LinAlg_normalize(int N, double x[N], double x_norm[N]){
    double size = LinAlg_vecnorm(N,x);
    for(int i = 0; i < N; i++){
        x_norm[i] = x[i]/size;
    }
}

int LinAlg_factorial(int k){
    int ret = 1;
    if(k > 0){
        for(int i = 1; i <= k; i++){
            ret *= i;
        }
    }else if (k < 0){
        LOG("Input to factorial() below 0\n");
        ret = -1;
    }
    return ret;
}

void LinAlg_expm3x3(double A[][3], double eA[][3], int k){
    int N = 3;
    LinAlg_eye(N,eA);
    double B[3][3];
    LinAlg_matcopy(N,N,A,B);
    double Bs[3][3]; //scaled B
    double scale = 0;
    for(int i = 1; i <= k; i++){
        scale = 1.0/LinAlg_factorial(i);
        LinAlg_matscalmult(N,N,B,scale,Bs); //Bs = scale*B;
        LinAlg_matmatadd(N,N,eA,Bs,eA);
        LinAlg_matmatmul_small(N,N,B,A,B);
    }
}

double LinAlg_det3x3(double A[][3]){
    double ret = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]);
    ret = ret - A[0][1]*(A[1][0]*A[2][2] - A[2][0]*A[1][2]);
    ret = ret + A[0][2]*(A[1][0]*A[2][1] - A[2][0]*A[1][1]);
    return ret;
}

void LinAlg_rotX(double t, double R[][3]){
    LinAlg_zeromat(3,3,R);
    R[0][0] = 1;
    R[1][1] = cos(t);
    R[2][2] = cos(t);
    R[2][1] = sin(t);
    R[1][2] = -sin(t);
}

void LinAlg_rotY(double t, double R[][3]){
    LinAlg_zeromat(3,3,R);
    R[1][1] = 1;
    R[0][0] = cos(t);
    R[2][2] = cos(t);
    R[0][2] = sin(t);
    R[2][0] = -sin(t);
}

void LinAlg_rotZ(double t, double R[][3]){
    LinAlg_zeromat(3,3,R);
    R[2][2] = 1;
    R[0][0] = cos(t);
    R[1][1] = cos(t);
    R[1][0] = sin(t);
    R[0][1] = -sin(t);
}

void LinAlg_printscal(double a){
    PRINTNUM("%.8f\n",a);
}

void LinAlg_printvec(int N, double v[N]){
    for(int i = 0; i < N; i++){
        PRINTNUM("[%.8f]\n",v[i]);
    }
    PRINT("\n");
}

void LinAlg_printmat(int N, int M, const double A[N][M]){
    for(int i = 0; i < N; i++){
        PRINT("[");
        for(int j = 0; j < M; j++){
            if(j < 2){
                PRINTNUM(" %.3f,",A[i][j]);
            }else{
               PRINTNUM(" %.3f",A[i][j]); 
            }
        }
        PRINT("]\n");
    }
    PRINT("\n");
}

void LinAlg_printvec_comma_separated(int N, double v[N]){
    for(int i = 0; i < N - 1; i++){
        PRINTNUM("%.8f,",v[i]);
    }
    PRINTNUM("%.8f",v[N - 1]);
    PRINT("\n");
}

void LinAlg_printvec_int(int N, int v[N]){
    for(int i = 0; i < N; i++){
        PRINTNUM("[%d]\n",v[i]);
    }
    PRINT("\n");
}

void LinAlg_find_parallel_vec(int N, double a[N], double b[N], double b_parr[N])
{
    //b_parr = b'*a*a/|a|^2
    double a_T_b = LinAlg_vecdot(N,a,b);
    double norm_a = LinAlg_vecnorm(N,a);
    LinAlg_vecscalmult(N,a,b_parr,a_T_b); //b_parr <- b'*a*a
    LinAlg_vecscalmult(N,b_parr,b_parr,1/(norm_a*norm_a)); //b_parr <- b_parr/|a|^2
}

void LinAlg_find_perpendicular_vec(int N, double a[N], double b[N], double b_perp[N])
{
    //b_perp = b - b_parr
    double b_parr[3]; //Can simply use b_perp here for memory saving
    LinAlg_find_parallel_vec(N,a,b,b_parr);
    LinAlg_vecvecsub(N,b,b_parr,b_perp);
}

double LinAlg_angle_between_vecs(int N, double a[N], double b[N], int returnDegrees)
{
    double theta = acos(LinAlg_vecdot(N,a,b)/(LinAlg_vecnorm(N,a)*LinAlg_vecnorm(N,b)));
    
    if(returnDegrees > 0){
        return theta*180/3.14159;
    }else if(returnDegrees == 0){
        return theta;
    }else{
        return 0;
    }
}

//New functions required by the solver

int LinAlg_PLU_decomposition_NXN_in_place(int N, int P[N], double LUMat[N][N])
{
    int P_local[N]; //Stack alloc, consider changing
    for(int i = 0; i < N; i++){
        P[i] = i; //Initialize the permutation vectors
        P_local[i] = i;
    }
    
    for(int diag_kk = 0; diag_kk < N - 1; diag_kk++){
        for(int i = 0; i < N; i++) P_local[i] = i; //Reset the local permutation vector
        LinAlg_find_permutation_vector(N, diag_kk, P_local, LUMat);
        LinAlg_switch_rows_with_P_below_diag(N, diag_kk, P_local, LUMat);
        LinAlg_kill_column_below_in_place(N, diag_kk, LUMat);
        LinAlg_update_global_permutation_vector(N, P_local, P);
    }
    //Add check the rank of U, do this by checking the fabs() of each pivot
    int rank = 0;
    for(int diag_kk = 0; diag_kk < N; diag_kk++){
        if(fabs(LUMat[diag_kk][diag_kk]) > 1.0E-7){
            rank++;
        }
    }
    return rank;
}

void LinAlg_switch_rows_with_P_below_diag(int N, int diag_kk, int P[N], double A[N][N])
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

void LinAlg_update_global_permutation_vector(int N, const int P_update[N], int P[N])
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
            PRINT("Error: Invalid P_update passed to update_global_permutation_vector; it attempts more than one row exchange, aborting\n");
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

void LinAlg_add_multiple_of_row_to_row(int N, int M, double A[N][M], int row_start_index, int from_row, int to_row, double k)
{
    if(from_row < 0 || from_row >= N || to_row < 0 || to_row >= N){
        PRINT("Error: from_rom/to_row outside of matrix dimenions in add_multiple_of_row_to_row, aborting\n");
        return;
    }

    for(int i = row_start_index; i < M; i++){
        A[to_row][i] += A[from_row][i]*k;
    }
}

void LinAlg_permute_vector_with_P(int N, int P[N], double b[N])
{
    //There are two ways to interpret the effect of P on b:
    //1: P[i] tells where element b[i] goes in the updated b. Ie: b[P[i]] = b[i]
    //2: P[i] tells which element of b goes to b[i].          Ie: b[i] = b[P[i]]

    //Example 1: b = [b0 b1 b2], P = [1 2 0] --> b_updated = [b2 b0 b1]
    //Example 2: b = [b0 b1 b2], P = [1 2 0] --> b_updated = [b1 b2 b0] <-- This is what we use

    //This part follows a chain of updates in the array and sets visited elements to -1 in P
    for(int j = 0; j < N; j++){
        if(P[j] != - 1 && P[j] != j){
            int i = j;
            int start_index = - 1;
            int permutation_index_set_to_negative = -1;
            int first_iteration = 1;
            double tmp;
            while(1){
                if(first_iteration){
                    first_iteration = 0;
                    start_index = i;
                    tmp = b[i];
                    b[i] = b[P[i]];
                }else{
                    permutation_index_set_to_negative = i;
                    i = P[i];
                    P[permutation_index_set_to_negative] = - 1;
                    if(P[i] != start_index){
                        b[i] = b[P[i]];
                    }else{
                        b[i] = tmp;
                        P[i] = - 1;
                        break;//End of the permutation chain
                    }
                }
            }
        }else{
            P[j] = - 1; //This is the case when P[j] = j or when P[j] = - 1. We mark it as visited
        }
    }
}

void LinAlg_kill_column_below_in_place(int N, int diag_kk, double LUMat[N][N])
{
    for(int k = diag_kk + 1; k < N; k++){
        double LUMat_kk = LUMat[diag_kk][diag_kk];
        if(LUMat_kk != 0){ //<----- This should be slightly larger than 0; eps
            double factor = - LUMat[k][diag_kk]/LUMat_kk;
            LinAlg_add_multiple_of_row_to_row(N,N,LUMat,diag_kk,diag_kk,k,factor);
            //We need to add the negative of factor to L in order to keep the expression unchanged
            //Remember that for in-place, we store both L and U in the same matrix
            LUMat[k][diag_kk] = - factor;
        }
    }
}

void LinAlg_find_permutation_vector(int N, int diag_kk, int P[N], double LUMat[N][N])
{
    if(diag_kk > (N - 2)){
        PRINT("Error: Invalid index for finding largest pivot! Aborting\n");
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

void LinAlg_matmatmul_no_alias(int N, int M, double A[N][M], double B[M][N], double C[N][N]){
    //A e NxM
    //B e MxN
    //C e NxN

    if(A == B || A == C || B == C){
        PRINT("Error: Aliasing not allowed in matmatmul_no_alias(...)! Aborting\n");
        return;
    }
    LinAlg_zeromat(N,N,C);
    for(int i = 0; i < N; i++){
        for(int j = 0; j < N; j++){
            for(int k = 0; k < M; k++){
                C[i][j] += A[i][k]*B[k][j]; //Some inefficiency with indexing I need to consider here
            }
        }
    }
}

void LinAlg_solve_lower_diagonal(int N, double L[N][N], double x[N], double b[N])
{
    //Solves for x in L*x=b
    for(int i = 0; i < N; i++){
        x[i] = b[i];
        for(int j = 0; j < i; j++){
            x[i] = x[i] - L[i][j]*x[j]; //We do not divide by L[i][i] as this is implicitly 1
        }
    }
}

void LinAlg_solve_upper_diagonal(int N, double U[N][N], double x[N], double b[N])
{
    //Solves for x in U*x=b
    for(int i = N - 1; i >= 0; i--){
        x[i] = b[i]/U[i][i];
        for(int j = i + 1; j <= N - 1; j++){
            x[i] = x[i] - U[i][j]*x[j]/U[i][i];
        }
    }
}

void LinAlg_solve_linear_system_NXN_in_place(int N, double A[N][N], double x[N], double b[N])
{
    //P*L*U*x = b
    //L*U*x = transpose(P)*b = c
    //U*x = y
    //L*y = c solve for y, then solve for x
    int P[N];
    double y[N];
    int rank = LinAlg_PLU_decomposition_NXN_in_place(N,P,A);
    if(rank < N){
        PRINT("System is rank-deficient!\n");
    }else{
        LinAlg_permute_vector_with_P(N,P,b);
        LinAlg_solve_lower_diagonal(N,A,y,b);
        LinAlg_solve_upper_diagonal(N,A,x,y);
    }
}
