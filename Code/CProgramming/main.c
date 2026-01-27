#include "stdio.h"
#include "stdlib.h"
#include "math.h"

//Struct for samples
typedef struct{
    double x;
    double y;
    double z;
}Sample;

void switch_rows_with_P_below_diag(int N, int diag_kk, int P[N], double A[N][N]);
void update_global_permutation_vector(int N, const int P_update[N], int P[N]);
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
double vecnorm(int N, double x[N]);
void zeromat(int N, int M,double A[N][M]); //Convert to mxn DONE
void matcopy(int N, int M, const double A[N][M], double B[N][M]); //Convert to mxn DONE
void matmatmul_small(int N, int M, double A[N][M], double B[M][N], double C[N][N]); //Convert to mxn DONE
void matmatmul_no_alias(int N, int M, double A[N][M], double B[M][N], double C[N][N]);
void matmatmatmul_NXN(int N, double A[N][N], double B[N][N], double C[N][N], double D[N][N]); //Convert to mxn. IGNORE, too much shit to do and only used in square case anyway
void matmatsub(int N, int M, double A[N][M], double B[N][M], double C[N][M]); //Convert to mxn DONE
void matmatadd(int N, int M, double A[N][M], double B[N][M], double C[N][M]);
void vecvecadd(int N, double a[N], double b[N], double c[N]);
void vecvecsub(int N, double a[N], double b[N], double c[N]);
void init_matrices(int N, double A[N][N]); //Convert to mxn. IGNORE, this is just some shit
void kill_column_below(int N, int diag_kk, double U[N][N], double L[N][N]);

void print_PLU_decomp(int N, double A[N][N], double P[N][N], double L[N][N], double U[N][N], int rank);
void solve_lower_diagonal(int N, double L[N][N], double x[N], double b[N]);
void solve_upper_diagonal(int N, double U[N][N], double x[N], double b[N]);
void solve_linear_system_NXN(int N, double A[N][N], double x[N], double b[N]);
void solve_linear_system_NXN_in_place(int N, double A[N][N], double x[N], double b[N]); //This solver modifies b!!
void matvecmul(int N, int M, double A[N][M], double x[M], double b[N]); //Convert to mxn DONE
void zerovec(int N, double x[N]);
void veccopy(int N, double a[N], double b[N]);
void mat_transpose(int N, int M, double A[N][M], double AT[M][N]); //Convert to mxn DONE
void matscalmult(int N, int M, double A[N][M], double k, double C[N][M]);
void vecscalmult(int N, double x[N], double y[N], double k);


//Takes in 
// - Optimization variable count,9 for ellipsoid. 
// - An array containing the variables
// - A magnetometer sample si of 3 parameters
// - This should probably be given a struct to work with as the argument lists are getting long
double evaluate_ri(int param_count, double opt_params[param_count], double si[3]);
void evaluate_r_vec(int param_count, int sample_count, double opt_params[param_count], Sample* samples, double r_vec[sample_count]);
void evaluate_gradient_ri(int param_count, double opt_params[param_count], double si[3], double grad_ri[param_count]);
void evaluate_jacobian_r(int param_count, int sample_count, double opt_params[param_count], Sample* samples, double J[sample_count][param_count]);
void evaluate_gradient_r(int param_count, int sample_count, double g[param_count], double JT[param_count][sample_count], double r_vec[sample_count]); //gradient g = J^T*r
void set_initial_guess_from_samples(int param_count, int sample_count, Sample* samples, double theta_0[param_count]); //Computes rough guess of the optimum solution
void LM_solver();
void get_magnetometer_calib(double theta[9], double correction_matrix[3][3], double correction_vector[3]);
void get_corrected_mag_vector(double correction_matrix[3][3], double correction_vector[3], Sample si, double m_corr[3]);

//Large variables for LM-solver
#define SAMPLE_COUNT 1000
#define PARAMETER_COUNT 9
double theta[PARAMETER_COUNT];
double d_theta[PARAMETER_COUNT]; //72B, parameter increment
double J[SAMPLE_COUNT][PARAMETER_COUNT]; //72kB
double JT[PARAMETER_COUNT][SAMPLE_COUNT]; //72kB
double JTJ[PARAMETER_COUNT][PARAMETER_COUNT]; //648B
double residue_vec[SAMPLE_COUNT]; //8kB
double gradient_vec[PARAMETER_COUNT];
Sample Samples[SAMPLE_COUNT];//24kB
//Total RAM required from this = 184.648kB, well within RP2350 RAM of 512kB, but optimizations can be done

int load_samples_from_file(const char* filepath, Sample* samples, int sample_count);

int main()
{
    LM_solver();
    printf("END OF PROGRAM\n");
    return 0;
}

int load_samples_from_file(const char* filepath, Sample* samples, int sample_count)
{
    FILE* fptr;
    fptr = fopen(filepath, "r");
    if(!fptr){
        printf("Failed to open file!\n");
        return 1;
    }
    
    for(int i = 0; i < sample_count; i++){
        if(fscanf(fptr,"%lf,%lf,%lf",&samples[i].x, &samples[i].y, &samples[i].z) != 3){
            printf("Read failed\n");
            fclose(fptr);
            return 2;
        }
    }
    fclose(fptr);
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
        update_global_permutation_vector(N, P_local, P);
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
    load_samples_from_file("../MagnetometerRawData/mag_cal.txt",Samples, sizeof(Samples)/sizeof(Sample));
    set_initial_guess_from_samples(PARAMETER_COUNT, SAMPLE_COUNT,Samples,theta);
    double lambda = 1E-4; //This worked in matlab
    double lambdaEye[PARAMETER_COUNT][PARAMETER_COUNT];
    double JTJ_plus_lambda_eye[PARAMETER_COUNT][PARAMETER_COUNT];
    int iter = 0;
    int max_iter = 1000;

    do{
        evaluate_r_vec(PARAMETER_COUNT,SAMPLE_COUNT, theta,Samples,residue_vec);
        evaluate_jacobian_r(PARAMETER_COUNT,SAMPLE_COUNT,theta,Samples,J);
        mat_transpose(SAMPLE_COUNT,PARAMETER_COUNT,J,JT);
        evaluate_gradient_r(PARAMETER_COUNT,SAMPLE_COUNT,gradient_vec,JT,residue_vec);
        matmatmul_no_alias(PARAMETER_COUNT,SAMPLE_COUNT,JT,J,JTJ);
        eye(PARAMETER_COUNT,lambdaEye);
        matscalmult(PARAMETER_COUNT,PARAMETER_COUNT,lambdaEye,lambda,lambdaEye);
        matmatadd(PARAMETER_COUNT,PARAMETER_COUNT,JTJ,lambdaEye,JTJ_plus_lambda_eye); //JT*J + lambda*I
        //update theta:
        vecscalmult(PARAMETER_COUNT,gradient_vec,gradient_vec, -1.0);
        solve_linear_system_NXN_in_place(PARAMETER_COUNT,JTJ_plus_lambda_eye,d_theta,gradient_vec);
        vecvecadd(PARAMETER_COUNT,d_theta,theta,theta);
        iter++;
    } while (vecnorm(PARAMETER_COUNT,d_theta) > 1E-6 && iter < max_iter);
    
    if(iter == max_iter){
        printf("Maximum allowed iterations of %d exceeded, some problem occured, probably bad initial estimate!\n", max_iter);
    }else{
        printf("Solution to the system at %d iterations with vecnorm(d_theta) = %.8f is T = \n", iter, vecnorm(PARAMETER_COUNT,d_theta));
        printvec(PARAMETER_COUNT, theta);
        // double mag_correction_matrix[3][3];
        // double mag_correction_vector[3];
        // get_magnetometer_calib(theta, mag_correction_matrix, mag_correction_vector);
        // printf("Correction matrix = \n");
        // printmat(3,3,mag_correction_matrix);
        // printf("Correction vector = \n");
        // printvec(3,mag_correction_vector);

        // //Print some corrected mag samples
        // double m_corr[3];
        // for(int i = 0; i < SAMPLE_COUNT; i+=50){
        //     get_corrected_mag_vector(mag_correction_matrix, mag_correction_vector, Samples[i], m_corr);
        //     printf("m_corr of norm %f is \n", vecnorm(3, m_corr));
        //     printvec(3,m_corr);
        // }
    }
}

void get_corrected_mag_vector(double correction_matrix[3][3], double correction_vector[3], Sample si, double m_corr[3])
{
    int N = 3;
    double m_raw[3];// = {si->x, si->y, si->z};
    m_raw[0] = si.x;
    m_raw[1] = si.y;
    m_raw[2] = si.z;
    vecvecsub(N,m_raw,correction_vector,m_corr); //m_corr = m_raw - correction_vector
    matvecmul(N,N,correction_matrix, m_corr, m_corr); //m_corr <- A*m_corr
}

void get_magnetometer_calib(double theta[9], double correction_matrix[3][3], double correction_vector[3])
{
    //theta = [Ba = 1/a^2 Bb = 1/b^2 Bc = 1/c^2 px py pz S0 S1 S2]
    //Cholesky decompose
    double L[3][3]; 
    L[0][0] = sqrt(theta[0]);
    L[1][0] = theta[6]/L[0][0];
    L[2][0] = theta[7]/L[0][0];
    L[1][1] = sqrt(theta[1] - L[1][0]*L[1][0]);
    L[2][1] = (theta[8] - L[1][0]*L[2][0])/L[1][1];
    L[2][2] = sqrt(theta[2] - L[2][0]*L[2][0] - L[2][1]*L[2][1]);
    mat_transpose(3,3,L,correction_matrix);
    correction_vector[0] = theta[3];
    correction_vector[1] = theta[4];
    correction_vector[2] = theta[5];
};

void set_initial_guess_from_samples(int param_count, int sample_count, Sample* samples, double theta_0[param_count])
{
    //Optimally the struct array should be sorted such that all values can be used, but rn we just use the largest and smallest value in each dimension.
    double max_x = 0, min_x = 0;
    double max_y = 0, min_y = 0;
    double max_z = 0, min_z = 0;
    double a_init, b_init, c_init, px_init, py_init, pz_init;

    for(int i = 0; i < sample_count; i++){
        if(samples[i].x > max_x){
            max_x = samples[i].x;
        }
        if(samples[i].x < min_x){
            min_x = samples[i].x;
        }

        if(samples[i].y > max_y){
            max_y = samples[i].y;
        }
        if(samples[i].y < min_y){
            min_y = samples[i].y;
        }

        if(samples[i].z > max_z){
            max_z = samples[i].z;
        }
        if(samples[i].z < min_z){
            min_z = samples[i].z;
        }
    }
    a_init = (max_x - min_x)/2;
    b_init = (max_y - min_y)/2;
    c_init = (max_z - min_z)/2;

    px_init = (max_x + min_x)/2;
    py_init = (max_y + min_y)/2;
    pz_init = (max_z + min_z)/2;

    theta_0[0] = 1/(a_init*a_init);
    theta_0[1] = 1/(b_init*b_init);
    theta_0[2] = 1/(c_init*c_init);

    theta_0[3] = px_init;
    theta_0[4] = py_init;
    theta_0[5] = pz_init;
    
    theta_0[6] = 0;
    theta_0[7] = 0;
    theta_0[8] = 0;
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

void evaluate_jacobian_r(int param_count, int sample_count, double opt_params[param_count], Sample* samples, double J[sample_count][param_count])
{
    //The jacobian of r_vec is a matrix where each row is a transposed gradient of ri
    double grad_ri[9];
    double si[3];
    for(int i = 0; i < sample_count; i++){
        si[0] = samples[i].x;
        si[1] = samples[i].y;
        si[2] = samples[i].z;
        evaluate_gradient_ri(param_count, opt_params, si, grad_ri);
        for(int j = 0; j < param_count; j++){
            J[i][j] = grad_ri[j];
        }
    }
}

void evaluate_r_vec(int param_count, int sample_count, double opt_params[param_count], Sample* samples, double r_vec[sample_count])
{
    double si[3];
    for(int i = 0; i < sample_count; i++){
        si[0] = samples[i].x;
        si[1] = samples[i].y;
        si[2] = samples[i].z;
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
    double y[N];
    int rank = PLU_decomposition_NXN_in_place(N,P,A);
    if(rank < N){
        printf("System is rank-deficient!\n");
    }else{
        permute_vector_with_P(N,P,b);
        solve_lower_diagonal(N,A,y,b);
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

double vecnorm(int N, double x[N]){
    double norm = 0;
    for(int i = 0; i < N; i++){
        norm += x[i]*x[i];
    }
    return sqrt(norm);
}

void vecvecadd(int N, double a[N], double b[N], double c[N]){
    for(int i = 0; i < N; i++){
        c[i] = a[i] + b[i];
    }
};

void vecvecsub(int N, double a[N], double b[N], double c[N]){
    for(int i = 0; i < N; i++){
        c[i] = a[i] - b[i];
    }
};

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
        x[i] = b[i];
        for(int j = 0; j < i; j++){
            x[i] = x[i] - L[i][j]*x[j]; //We do not divide by L[i][i] as this is implicitly 1
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
    zeromat(N,N,C);
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
        printf("[%.8f]\n",v[i]);
    }
    printf("\n");
}

void printvec_int(int N, int v[N]){
    for(int i = 0; i < N; i++){
        printf("[%d]\n",v[i]);
    }
    printf("\n");
}

