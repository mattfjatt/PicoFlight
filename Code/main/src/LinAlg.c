#include "headers/linalg.h"

void linalg_zeromat(int n, int m,double A[n][m]){
    for(int i = 0; i < n; i++){
        for(int j = 0; j < m; j++){
            A[i][j]= 0;
        }
    }
}

void linalg_zerovec(int n, double x[n]){
    for(int i = 0; i < n; i++){
        x[i] = 0;
    }
}

void linalg_eye(int n,double A[n][n])
{
    linalg_zeromat(n, n, A);
    for(int i = 0; i < n; i++){
        A[i][i]= 1;
    }
}

void linalg_matvecmul(int n, int m, double A[n][m], double x[m], double b[n])
{
    if(n == m){
        double temp[n];
        linalg_zerovec(n,temp); //Found no cheaper way to do this than to create an entire temp vector
        for(int i = 0; i < n; i++){
            for(int j = 0; j < m; j++){
                temp[i] += A[i][j]*x[j];
            }
        }
        linalg_veccopy(n,temp,b);
    }else{
        if(x == b){
            LOG("Error: Input and output vector can not be the same address in memory for n != m, aborting\n");
            return;
        }
        linalg_zerovec(n,b);
        for(int i = 0; i < n; i++){
            for(int j = 0; j < m; j++){
                b[i] += A[i][j]*x[j];
            }
        }
    }
}

void linalg_matmatmul_small(int n, int m, double A[n][m], double B[m][n], double C[n][n]){
    //A e NxM
    //B e MxN
    //C e NxN
    //Creating a temp matrix on the stack is a problem if n is very large, but even the Jacobian matrices of
    //J e 1000x9 and JT e 9x1000 will only yield JTJ e 9x9 when multiplied, this is 648 bytes, and the stack is 8kB 
    //But this is unnecessary as C will not be an alias anyway when n != m
    double temp[n][n];
    linalg_zeromat(n, n, temp); //temp set to be all zeroes
    for(int i = 0; i < n; i++){
        for(int j = 0; j < n; j++){
            for(int k = 0; k < m; k++){
                temp[i][j] += A[i][k]*B[k][j]; //Some inefficiency with indexing I need to consider here
            }
        }
    }
    linalg_matcopy(n, n, temp, C);
}

void linalg_vecvecadd(int n, double a[n], double b[n], double c[n]){
    for(int i = 0; i < n; i++){
        c[i] = a[i] + b[i];
    }
}

void linalg_vecvecsub(int n, double a[n], double b[n], double c[n]){
    for(int i = 0; i < n; i++){
        c[i] = a[i] - b[i];
    }
}

void linalg_vecscalmult(int n, double x[n], double y[n], double k){
    for(int i = 0; i < n; i++){
        y[i] = k*x[i];
    }
}

double linalg_vecdot(int n, double a[n], double b[n]){
    double c = 0;
    for(int i = 0; i < n; i++){
        c += a[i]*b[i];
    }
    return c;
}

void linalg_matmatadd(int n, int m, double A[n][m], double B[n][m], double C[n][m]){
    for(int i = 0; i < n; i++){
        for(int j = 0; j < m; j++){
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

void linalg_matmatsub(int n, int m, double A[n][m], double B[n][m], double C[n][m]){
    for(int i = 0; i < n; i++){
        for(int j = 0; j < m; j++){
            C[i][j] = A[i][j] - B[i][j];
        }
    }
}

void linalg_matscalmult(int n, int m, double A[n][m], double k, double C[n][m])
{
    for(int i = 0; i < n; i++){
        for(int j = 0; j < m; j++){
            C[i][j] = k*A[i][j];
        }
    }
}

void linalg_mat2colvecs3x3(double R[][3], double r1[3], double r2[3], double r3[3]){
    for(int i = 0; i < 3; i++){
        r1[i] = R[i][0];
        r2[i] = R[i][1];
        r3[i] = R[i][2];
    }
}

void linalg_colvecs2mat3x3(double R[][3], double r1[3], double r2[3], double r3[3]){
    for(int i = 0; i < 3; i++){
        R[i][0] = r1[i];
        R[i][1] = r2[i];
        R[i][2] = r3[i];
    }
}

void linalg_matnormalizerotation(double R[][3]){
    //Crude method of ensuring R remains in SO(3)
    int n = 3;
    double r1[3], r2[3], r3[3];
    double u1[3], u2[3], u3[3];
    double k1, k2, k3;
    double k1u1[3], k2u1[3], k3u2[3];
    linalg_mat2colvecs3x3(R, r1, r2, r3);
    //u1 = r1;
    linalg_veccopy(n,r1,u1);

    //u2 = r2 - k1*u1
    //k1 = (r2*u1)/(u1*u1)
    k1 = linalg_vecdot(n,r2,u1)/linalg_vecdot(n,u1,u1);
    linalg_vecscalmult(n,u1,k1u1, k1);
    linalg_vecvecsub(n,r2,k1u1,u2);

    //u3 = r3 - (r3*u1)/(u1*u1)*u1 - (r3*u2)/(u2*u2)*u2
    //u3 = r3 - k2*u1 - k3*u2;
    //k2 = (r3*u1)/(u1*u1)
    //k3 = (r3*u2)/(u2*u2)
    k2 = linalg_vecdot(n,r3,u1)/linalg_vecdot(n,u1,u1);
    k3 = linalg_vecdot(n,r3,u2)/linalg_vecdot(n,u2,u2);
    linalg_vecscalmult(n,u1,k2u1, k2);
    linalg_vecscalmult(n,u2,k3u2, k3);
    linalg_veccopy(n,r3,u3);
    linalg_vecvecsub(n,u3,k2u1,u3);
    linalg_vecvecsub(n,u3,k3u2,u3);

    //Normalize the orthogonal vectors 
    linalg_normalize(n,u1,u1);
    linalg_normalize(n,u2,u2);
    linalg_normalize(n,u3,u3);
    
    linalg_colvecs2mat3x3(R,u1,u2,u3);

}

void linalg_vec2skew3x3(double x[3], double S[][3]){
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

void linalg_matcopy(int n, int m, const double A[n][m], double B[n][m]){
    for(int i = 0; i < n; i++){
        for(int j = 0; j < m; j++){
            B[i][j] = A[i][j];
        }
    }
}

void linalg_veccopy(int n, double a[n], double b[n]){
    for(int i = 0; i < n; i++){
        b[i] = a[i];
    }
}

void linalg_mattranspose(int n, int m, double A[n][m], double AT[m][n])
{
    if(n == m){
        for(int i = 0; i < n; i++){
            for(int j = i; j < m; j++){
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
            for(int i = 0; i < n; i++){
                for(int j = 0; j < m; j++){
                    AT[j][i] = A[i][j];
                }
            }
        }
    }
}

double linalg_vecnorm(int n, double x[n]){
    double norm = 0;
    for(int i = 0; i < n; i++){
        norm += x[i]*x[i];
    }
    return sqrt(norm);
}

void linalg_normalize(int n, double x[n], double x_norm[n]){
    double size = linalg_vecnorm(n,x);
    for(int i = 0; i < n; i++){
        x_norm[i] = x[i]/size;
    }
}

int linalg_factorial(int k){
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

void linalg_expm3x3(double A[][3], double eA[][3], int k){
    int n = 3;
    linalg_eye(n,eA);
    double B[3][3];
    linalg_matcopy(n,n,A,B);
    double Bs[3][3]; //scaled B
    double scale = 0;
    for(int i = 1; i <= k; i++){
        scale = 1.0/linalg_factorial(i);
        linalg_matscalmult(n,n,B,scale,Bs); //Bs = scale*B;
        linalg_matmatadd(n,n,eA,Bs,eA);
        linalg_matmatmul_small(n,n,B,A,B);
    }
}

double linalg_det3x3(double A[][3]){
    double ret = A[0][0]*(A[1][1]*A[2][2] - A[1][2]*A[2][1]);
    ret = ret - A[0][1]*(A[1][0]*A[2][2] - A[2][0]*A[1][2]);
    ret = ret + A[0][2]*(A[1][0]*A[2][1] - A[2][0]*A[1][1]);
    return ret;
}

void linalg_rotX(double t, double R[][3]){
    linalg_zeromat(3,3,R);
    R[0][0] = 1;
    R[1][1] = cos(t);
    R[2][2] = cos(t);
    R[2][1] = sin(t);
    R[1][2] = -sin(t);
}

void linalg_rotY(double t, double R[][3]){
    linalg_zeromat(3,3,R);
    R[1][1] = 1;
    R[0][0] = cos(t);
    R[2][2] = cos(t);
    R[0][2] = sin(t);
    R[2][0] = -sin(t);
}

void linalg_rotZ(double t, double R[][3]){
    linalg_zeromat(3,3,R);
    R[2][2] = 1;
    R[0][0] = cos(t);
    R[1][1] = cos(t);
    R[1][0] = sin(t);
    R[0][1] = -sin(t);
}

void linalg_printscal(double a){
    PRINTNUM("%.8f\n",a);
}

void linalg_printvec(int n, double v[n]){
    for(int i = 0; i < n; i++){
        PRINTNUM("[%.8f]\n",v[i]);
    }
    PRINT("\n");
}

void linalg_printmat(int n, int m, const double A[n][m]){
    for(int i = 0; i < n; i++){
        PRINT("[");
        for(int j = 0; j < m; j++){
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

void linalg_printvec_comma_separated(int n, double v[n]){
    for(int i = 0; i < n - 1; i++){
        PRINTNUM("%.8f,",v[i]);
    }
    PRINTNUM("%.8f",v[n - 1]);
    PRINT("\n");
}

void linalg_printvec_int(int n, int v[n]){
    for(int i = 0; i < n; i++){
        PRINTNUM("[%d]\n",v[i]);
    }
    PRINT("\n");
}

void linalg_find_parallel_vec(int n, double a[n], double b[n], double b_parr[n])
{
    //b_parr = b'*a*a/|a|^2
    double a_T_b = linalg_vecdot(n,a,b);
    double norm_a = linalg_vecnorm(n,a);
    linalg_vecscalmult(n,a,b_parr,a_T_b); //b_parr <- b'*a*a
    linalg_vecscalmult(n,b_parr,b_parr,1/(norm_a*norm_a)); //b_parr <- b_parr/|a|^2
}

void linalg_find_perpendicular_vec(int n, double a[n], double b[n], double b_perp[n])
{
    //b_perp = b - b_parr
    double b_parr[3]; //Can simply use b_perp here for memory saving
    linalg_find_parallel_vec(n,a,b,b_parr);
    linalg_vecvecsub(n,b,b_parr,b_perp);
}

double linalg_angle_between_vecs(int n, double a[n], double b[n], int return_degrees)
{
    double theta = acos(linalg_vecdot(n,a,b)/(linalg_vecnorm(n,a)*linalg_vecnorm(n,b)));
    
    if(return_degrees > 0){
        return theta*180/3.14159;
    }else if(return_degrees == 0){
        return theta;
    }else{
        return 0;
    }
}

//New functions required by the solver

int linalg_plu_decomposition_square_in_place(int n, int P[n], double LU_mat[n][n])
{
    int P_local[n]; //Stack alloc, consider changing
    for(int i = 0; i < n; i++){
        P[i] = i; //Initialize the permutation vectors
        P_local[i] = i;
    }
    
    for(int diag_kk = 0; diag_kk < n - 1; diag_kk++){
        for(int i = 0; i < n; i++) P_local[i] = i; //Reset the local permutation vector
        linalg_find_permutation_vector(n, diag_kk, P_local, LU_mat);
        linalg_switch_rows_with_permutation_vector_below_diag(n, diag_kk, P_local, LU_mat);
        linalg_kill_column_below_in_place(n, diag_kk, LU_mat);
        linalg_update_global_permutation_vector(n, P_local, P);
    }
    //Add check the rank of U, do this by checking the fabs() of each pivot
    int rank = 0;
    for(int diag_kk = 0; diag_kk < n; diag_kk++){
        if(fabs(LU_mat[diag_kk][diag_kk]) > 1.0E-7){
            rank++;
        }
    }
    return rank;
}

void linalg_switch_rows_with_permutation_vector_below_diag(int n, int diag_kk, int P[n], double A[n][n])
{
    int m = n;

    for(int i = diag_kk; i < n; i++){ //Only check/switch rows starting from the diagonal index
        if(P[i] != i){
            for(int j = 0; j < m; j++){
                double tmp = A[i][j];
                A[i][j] = A[P[i]][j];
                A[P[i]][j] = tmp;
            }
            break; //Exit the loop when a row swap has been performed
        }
    }
}

void linalg_update_global_permutation_vector(int n, const int P_update[n], int P[n])
{
    //This function is only designed to accomododate one row exchange!
    //I.e P_update can not be equal to [1 0 3 2] as this corresponds to 2 row exchanges
    int index_mismatch = 0;
    const int max_switches = 2;
    for(int i = 0; i < n; i++){
        if(P_update[i] != i){
            index_mismatch++;
        }
        if(index_mismatch > max_switches){
            PRINT("Error: Invalid P_update passed to update_global_permutation_vector; it attempts more than one row exchange, aborting\n");
            return;
        }
    }

    for(int i = 0; i < n; i++){
        if(P_update[i] != i){ //If P_update[i] = i, no rows should change
            int tmp = P[i];
            P[i] = P[P_update[i]];
            P[P_update[i]] = tmp;
            break;
        }
    }
}

void linalg_add_multiple_of_row_to_row(int n, int m, double A[n][m], int row_start_index, int from_row, int to_row, double k)
{
    if(from_row < 0 || from_row >= n || to_row < 0 || to_row >= n){
        PRINT("Error: from_rom/to_row outside of matrix dimenions in add_multiple_of_row_to_row, aborting\n");
        return;
    }

    for(int i = row_start_index; i < m; i++){
        A[to_row][i] += A[from_row][i]*k;
    }
}

void linalg_permute_vector_with_permutation_vector(int n, int P[n], double b[n])
{
    //There are two ways to interpret the effect of P on b:
    //1: P[i] tells where element b[i] goes in the updated b. Ie: b[P[i]] = b[i]
    //2: P[i] tells which element of b goes to b[i].          Ie: b[i] = b[P[i]]

    //Example 1: b = [b0 b1 b2], P = [1 2 0] --> b_updated = [b2 b0 b1]
    //Example 2: b = [b0 b1 b2], P = [1 2 0] --> b_updated = [b1 b2 b0] <-- This is what we use

    //This part follows a chain of updates in the array and sets visited elements to -1 in P
    for(int j = 0; j < n; j++){
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

void linalg_kill_column_below_in_place(int n, int diag_kk, double LU_mat[n][n])
{
    for(int k = diag_kk + 1; k < n; k++){
        double LUMat_kk = LU_mat[diag_kk][diag_kk];
        if(LUMat_kk != 0){ //<----- This should be slightly larger than 0; eps
            double factor = - LU_mat[k][diag_kk]/LUMat_kk;
            linalg_add_multiple_of_row_to_row(n,n,LU_mat,diag_kk,diag_kk,k,factor);
            //We need to add the negative of factor to L in order to keep the expression unchanged
            //Remember that for in-place, we store both L and U in the same matrix
            LU_mat[k][diag_kk] = - factor;
        }
    }
}

void linalg_find_permutation_vector(int n, int diag_kk, int P[n], double LU_mat[n][n])
{
    if(diag_kk > (n - 2)){
        PRINT("Error: Invalid index for finding largest pivot! Aborting\n");
        return;
    }

    //P works in such a way that P = [0 1 2] corresponds to no row changes
    //P = [1 0 2] corresponds to row 1 being in 0th position and row 0 in 1st position

    //Find the largest element below and including diagonal element diag_kk
    double largest = 0.0;
    int largest_element_index = 0;
    for(int below_diag_kk = diag_kk; below_diag_kk < n; below_diag_kk++){
        double element = fabs(LU_mat[below_diag_kk][diag_kk]);
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

void linalg_matmatmul_no_alias(int n, int m, double A[n][m], double B[m][n], double C[n][n]){
    //A e NxM
    //B e MxN
    //C e NxN

    if(A == B || A == C || B == C){
        PRINT("Error: Aliasing not allowed in matmatmul_no_alias(...)! Aborting\n");
        return;
    }
    linalg_zeromat(n,n,C);
    for(int i = 0; i < n; i++){
        for(int j = 0; j < n; j++){
            for(int k = 0; k < m; k++){
                C[i][j] += A[i][k]*B[k][j]; //Some inefficiency with indexing I need to consider here
            }
        }
    }
}

void linalg_solve_lower_diagonal(int n, double L[n][n], double x[n], double b[n])
{
    //Solves for x in L*x=b
    for(int i = 0; i < n; i++){
        x[i] = b[i];
        for(int j = 0; j < i; j++){
            x[i] = x[i] - L[i][j]*x[j]; //We do not divide by L[i][i] as this is implicitly 1
        }
    }
}

void linalg_solve_upper_diagonal(int n, double U[n][n], double x[n], double b[n])
{
    //Solves for x in U*x=b
    for(int i = n - 1; i >= 0; i--){
        x[i] = b[i]/U[i][i];
        for(int j = i + 1; j <= n - 1; j++){
            x[i] = x[i] - U[i][j]*x[j]/U[i][i];
        }
    }
}

void linalg_solve_linear_system_square_in_place(int n, double A[n][n], double x[n], double b[n])
{
    //P*L*U*x = b
    //L*U*x = transpose(P)*b = c
    //U*x = y
    //L*y = c solve for y, then solve for x
    int P[n];
    double y[n];
    int rank = linalg_plu_decomposition_square_in_place(n,P,A);
    if(rank < n){
        PRINT("System is rank-deficient!\n");
    }else{
        linalg_permute_vector_with_permutation_vector(n,P,b);
        linalg_solve_lower_diagonal(n,A,y,b);
        linalg_solve_upper_diagonal(n,A,x,y);
    }
}
