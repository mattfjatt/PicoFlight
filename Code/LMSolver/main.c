#include "stdio.h"
#include "stdlib.h"
#include "math.h"

//Struct for samples
typedef struct{
    double x;
    double y;
    double z;
}Sample;

//------Old functions that are updated and belong in LinAlg------
void LinAlg_printmat(int N, int M, const double A[N][M]);                                   //Done
void LinAlg_printvec(int N, double v[N]);                                                   //Done
void LinAlg_eye(int N,double A[N][N]);                                                      //Done
double LinAlg_vecnorm(int N, double x[N]);                                                  //Done
void LinAlg_zeromat(int N, int M,double A[N][M]);                                           //Done
void LinAlg_matcopy(int N, int M, const double A[N][M], double B[N][M]);                    //Done
void LinAlg_matmatmul_small(int N, int M, double A[N][M], double B[M][N], double C[N][N]);  //Done, replaces matmatmul
void LinAlg_matmatsub(int N, int M, double A[N][M], double B[N][M], double C[N][M]);        //Done
void LinAlg_matmatadd(int N, int M, double A[N][M], double B[N][M], double C[N][M]);        //Done
void LinAlg_vecvecadd(int N, double a[N], double b[N], double c[N]);                        //Done
void LinAlg_vecvecsub(int N, double a[N], double b[N], double c[N]);                        //Done
void LinAlg_matvecmul(int N, int M, double A[N][M], double x[M], double b[N]);              //Done
void LinAlg_zerovec(int N, double x[N]);                                                    //Done
void LinAlg_veccopy(int N, double a[N], double b[N]);                                       //Done
void LinAlg_mattranspose(int N, int M, double A[N][M], double AT[M][N]);                    //Done, replaces LinAlg_transpose
void LinAlg_matscalmult(int N, int M, double A[N][M], double k, double C[N][M]);            //Done
void LinAlg_vecscalmult(int N, double x[N], double y[N], double k);                         //Done
//-----------------------------End-------------------------------

//--------------New functions that belong in LinAlg--------------
void LinAlg_printvec_int(int N, int v[N]);                                                  //Done
int LinAlg_PLU_decomposition_NXN_in_place(int N, int P[N], double LUMat[N][N]);             //Done
void LinAlg_switch_rows_with_P_below_diag(int N, int diag_kk, int P[N], double A[N][N]);    //Done
void LinAlg_update_global_permutation_vector(int N, const int P_update[N], int P[N]);       //Done
void LinAlg_add_multiple_of_row_to_row(int N, int M, double A[N][M], int row_start_index, int from_row, int to_row, double k); //Done
void LinAlg_permute_vector_with_P(int N, int P[N], double b[N]);                            //Done
void LinAlg_kill_column_below_in_place(int N, int diag_kk, double LUMat[N][N]);             //Done
void LinAlg_find_permutation_vector(int N, int diag_kk, int P[N], double LUMat[N][N]);      //Done
void LinAlg_matmatmul_no_alias(int N, int M, double A[N][M], double B[M][N], double C[N][N]); //Done, used for larger matrices most likely defined globally
void LinAlg_solve_lower_diagonal(int N, double L[N][N], double x[N], double b[N]);          //Done
void LinAlg_solve_upper_diagonal(int N, double U[N][N], double x[N], double b[N]);          //Done
void LinAlg_solve_linear_system_NXN_in_place(int N, double A[N][N], double x[N], double b[N]); //Done, this solver modifies b!!
//-----------------------------End-------------------------------



//--------Make a new source file for the optimizer?--------------
//Large variables for LM-solver
#define SAMPLE_COUNT 687 //1000
#define PARAMETER_COUNT 9

// typedef struct {
//     double theta[PARAMETER_COUNT];
//     double d_theta[PARAMETER_COUNT]; //72B, parameter increment
//     double J[SAMPLE_COUNT][PARAMETER_COUNT]; //72kB
//     double JT[PARAMETER_COUNT][SAMPLE_COUNT]; //72kB
//     double JTJ[PARAMETER_COUNT][PARAMETER_COUNT]; //648B
//     double residue_vec[SAMPLE_COUNT]; //8kB
//     double gradient_vec[PARAMETER_COUNT];
//     Sample Samples[SAMPLE_COUNT];//24kB
// }OptStruct;

// OptStruct optimizationData;

double Namespace_evaluate_ri(int param_count, double opt_params[param_count], double si[3]);                                        //Done
void Namespace_evaluate_gradient_ri(int param_count, double opt_params[param_count], double si[3], double grad_ri[param_count]);    //Done
//According to my current unerstanding of my own implementation, the two functions above are the only functions strictly specific to the magnetometer cost function
//To generalize, they could be passed as function pointers to the optimizer below, this means ri and gradient_ri can be defined and declared in the sensor-specific file,
//while the rest of the optimizer can be in its own optimizer namespace.

void Namespace_evaluate_r_vec(int param_count, int sample_count, double opt_params[param_count], Sample* samples, double r_vec[sample_count]); //Done
void Namespace_evaluate_jacobian_r(int param_count, int sample_count, double opt_params[param_count], Sample* samples, double J[sample_count][param_count]); //Done
void Namespace_evaluate_gradient_r(int param_count, int sample_count, double g[param_count], double JT[param_count][sample_count], double r_vec[sample_count]); //Done, gradient g = J^T*r
void Namespace_set_initial_guess_from_samples(int param_count, int sample_count, Sample* samples, double theta_0[param_count]); //Done, computes rough guess of the optimum solution
void Namespace_LM_solver(); //Done
void Namespace_get_magnetometer_calib(double theta[9], double correction_matrix[3][3], double correction_vector[3]); //Done
void Namespace_get_corrected_mag_vector(double correction_matrix[3][3], double correction_vector[3], Sample si, double m_corr[3]); //Done



double theta[PARAMETER_COUNT];
double d_theta[PARAMETER_COUNT]; //72B, parameter increment
double J[SAMPLE_COUNT][PARAMETER_COUNT]; //72kB
double JT[PARAMETER_COUNT][SAMPLE_COUNT]; //72kB
double JTJ[PARAMETER_COUNT][PARAMETER_COUNT]; //648B
double residue_vec[SAMPLE_COUNT]; //8kB
double gradient_vec[PARAMETER_COUNT];
Sample Samples[SAMPLE_COUNT];//24kB
//Total RAM required from this = 184.648kB, well within RP2350 RAM of 512kB, but optimizations can be done
//-----------------------------End-------------------------------


int load_samples_from_file(const char* filepath, Sample* samples, int sample_count);
int save_samples_as_c_array(const char* filename, Sample* samples, int sample_count, const char* array_name);


int main()
{
    load_samples_from_file("../MagnetometerRawData/MMC5603_data2.txt",Samples, sizeof(Samples)/sizeof(Sample));
    save_samples_as_c_array("../MagnetometerRawData/MMC5603_array.txt", Samples,SAMPLE_COUNT, "magnetometer_samples_MMC5603");
    Namespace_LM_solver();
    
    // printf("File path = %s. Line number = %d. Function name = %s\n", __FILE__, __LINE__, __func__);


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
    return 0;
}

int save_samples_as_c_array(const char* filename, Sample* samples, int sample_count, const char* array_name)
{
    FILE* fptr = fopen(filename, "w");
    if (!fptr) {
        printf("Failed to open output file!\n");
        return 1;
    }

    fprintf(fptr, "static const Sample %s[%d] = {\n", array_name, sample_count);
    for (int i = 0; i < sample_count; i++) {
        fprintf(fptr, "    {%.8f, %.8f, %.8f}%s\n",
                samples[i].x, samples[i].y, samples[i].z,
                (i < sample_count - 1) ? "," : "");
    }
    fprintf(fptr, "};\n");

    fclose(fptr);
    return 0;
}

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

void LinAlg_find_permutation_vector(int N, int diag_kk, int P[N], double LUMat[N][N])
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

void Namespace_LM_solver()
{
    //load_samples_from_file("../MagnetometerRawData/MMC5603_data2.txt",Samples, sizeof(Samples)/sizeof(Sample));
    Namespace_set_initial_guess_from_samples(PARAMETER_COUNT, SAMPLE_COUNT,Samples,theta);
    double lambda = 1E-4; //This worked in matlab
    double lambdaEye[PARAMETER_COUNT][PARAMETER_COUNT];
    double JTJ_plus_lambda_eye[PARAMETER_COUNT][PARAMETER_COUNT];
    int iter = 0;
    int max_iter = 1000;

    do{
        Namespace_evaluate_r_vec(PARAMETER_COUNT,SAMPLE_COUNT, theta,Samples,residue_vec);
        Namespace_evaluate_jacobian_r(PARAMETER_COUNT,SAMPLE_COUNT,theta,Samples,J);

        
        LinAlg_mattranspose(SAMPLE_COUNT,PARAMETER_COUNT,J,JT);
        Namespace_evaluate_gradient_r(PARAMETER_COUNT,SAMPLE_COUNT,gradient_vec,JT,residue_vec);
        LinAlg_matmatmul_no_alias(PARAMETER_COUNT,SAMPLE_COUNT,JT,J,JTJ);
        LinAlg_eye(PARAMETER_COUNT,lambdaEye);
        LinAlg_matscalmult(PARAMETER_COUNT,PARAMETER_COUNT,lambdaEye,lambda,lambdaEye);
        LinAlg_matmatadd(PARAMETER_COUNT,PARAMETER_COUNT,JTJ,lambdaEye,JTJ_plus_lambda_eye); //JT*J + lambda*I
        //update theta:
        LinAlg_vecscalmult(PARAMETER_COUNT,gradient_vec,gradient_vec, -1.0);
        LinAlg_solve_linear_system_NXN_in_place(PARAMETER_COUNT,JTJ_plus_lambda_eye,d_theta,gradient_vec);
        LinAlg_vecvecadd(PARAMETER_COUNT,d_theta,theta,theta);
        iter++;
    } while (LinAlg_vecnorm(PARAMETER_COUNT,d_theta) > 1E-6 && iter < max_iter);
    
    if(iter == max_iter){
        printf("Maximum allowed iterations of %d exceeded, some problem occured, probably bad initial estimate!\n", max_iter);
    }else{
        printf("Solution to the system at %d iterations with vecnorm(d_theta) = %.8f is T = \n", iter, LinAlg_vecnorm(PARAMETER_COUNT,d_theta));
        LinAlg_printvec(PARAMETER_COUNT, theta);
        // double mag_correction_matrix[3][3];
        // double mag_correction_vector[3];
        // Namespace_get_magnetometer_calib(theta, mag_correction_matrix, mag_correction_vector);
        // printf("Correction matrix = \n");
        // LinAlg_printmat(3,3,mag_correction_matrix);
        // printf("Correction vector = \n");
        // LinAlg_printvec(3,mag_correction_vector);

        // //Print some corrected mag samples
        // double m_corr[3];
        // for(int i = 0; i < SAMPLE_COUNT; i+=50){
        //     Namespace_get_corrected_mag_vector(mag_correction_matrix, mag_correction_vector, Samples[i], m_corr);
        //     printf("m_corr of norm %f is \n", vecnorm(3, m_corr));
        //     LinAlg_printvec(3,m_corr);
        // }
    }
}

void Namespace_get_corrected_mag_vector(double correction_matrix[3][3], double correction_vector[3], Sample si, double m_corr[3])
{
    int N = 3;
    double m_raw[3];// = {si->x, si->y, si->z};
    m_raw[0] = si.x;
    m_raw[1] = si.y;
    m_raw[2] = si.z;
    LinAlg_vecvecsub(N,m_raw,correction_vector,m_corr); //m_corr = m_raw - correction_vector
    LinAlg_matvecmul(N,N,correction_matrix, m_corr, m_corr); //m_corr <- A*m_corr
}

void Namespace_get_magnetometer_calib(double theta[9], double correction_matrix[3][3], double correction_vector[3])
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
    LinAlg_mattranspose(3,3,L,correction_matrix);
    correction_vector[0] = theta[3];
    correction_vector[1] = theta[4];
    correction_vector[2] = theta[5];
};

void Namespace_set_initial_guess_from_samples(int param_count, int sample_count, Sample* samples, double theta_0[param_count])
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

void Namespace_evaluate_gradient_r(int param_count, int sample_count, double g[param_count], double JT[param_count][sample_count], double r_vec[sample_count])
{
    //g = JT*r
    LinAlg_zerovec(param_count,g);
    for(int i = 0; i < param_count; i++){
        for(int j = 0; j < sample_count; j++){
            g[i] += JT[i][j]*r_vec[j];
        }
    }
}

void Namespace_evaluate_gradient_ri(int param_count, double opt_params[param_count], double si[3], double grad_ri[param_count])
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

void Namespace_evaluate_jacobian_r(int param_count, int sample_count, double opt_params[param_count], Sample* samples, double J[sample_count][param_count])
{
    //The jacobian of r_vec is a matrix where each row is a transposed gradient of ri
    double grad_ri[9];
    double si[3];
    for(int i = 0; i < sample_count; i++){
        si[0] = samples[i].x;
        si[1] = samples[i].y;
        si[2] = samples[i].z;
        Namespace_evaluate_gradient_ri(param_count, opt_params, si, grad_ri);
        for(int j = 0; j < param_count; j++){
            J[i][j] = grad_ri[j];
        }
    }
}

void Namespace_evaluate_r_vec(int param_count, int sample_count, double opt_params[param_count], Sample* samples, double r_vec[sample_count])
{
    double si[3];
    for(int i = 0; i < sample_count; i++){
        si[0] = samples[i].x;
        si[1] = samples[i].y;
        si[2] = samples[i].z;
        r_vec[i] = Namespace_evaluate_ri(param_count, opt_params, si);
    }
}

double Namespace_evaluate_ri(int param_count, double opt_params[param_count], double si[3])
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
        printf("System is rank-deficient!\n");
    }else{
        LinAlg_permute_vector_with_P(N,P,b);
        LinAlg_solve_lower_diagonal(N,A,y,b);
        LinAlg_solve_upper_diagonal(N,A,x,y);
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
            printf("Error: Input and output vector can not be the same address in memory for N != M, aborting\n");
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

double LinAlg_vecnorm(int N, double x[N]){
    double norm = 0;
    for(int i = 0; i < N; i++){
        norm += x[i]*x[i];
    }
    return sqrt(norm);
}

void LinAlg_vecvecadd(int N, double a[N], double b[N], double c[N]){
    for(int i = 0; i < N; i++){
        c[i] = a[i] + b[i];
    }
};

void LinAlg_vecvecsub(int N, double a[N], double b[N], double c[N]){
    for(int i = 0; i < N; i++){
        c[i] = a[i] - b[i];
    }
};

void LinAlg_vecscalmult(int N, double x[N], double y[N], double k){
    for(int i = 0; i < N; i++){
        y[i] = k*x[i];
    }
}

void LinAlg_zerovec(int N, double x[N]){
    for(int i = 0; i < N; i++){
        x[i] = 0;
    }
}

void LinAlg_veccopy(int N, double a[N], double b[N]){
    for(int i = 0; i < N; i++){
        b[i] = a[i];
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

void LinAlg_matscalmult(int N, int M, double A[N][M], double k, double C[N][M])
{
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            C[i][j] = k*A[i][j];
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

void LinAlg_matmatmul_no_alias(int N, int M, double A[N][M], double B[M][N], double C[N][N]){
    //A e NxM
    //B e MxN
    //C e NxN

    if(A == B || A == C || B == C){
        printf("Error: Aliasing not allowed in matmatmul_no_alias(...)!\n");
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

void LinAlg_matmatsub(int N, int M, double A[N][M], double B[N][M], double C[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            C[i][j] = A[i][j] - B[i][j];
        }
    }
}

void LinAlg_matmatadd(int N, int M, double A[N][M], double B[N][M], double C[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            C[i][j] = A[i][j] + B[i][j];
        }
    }
}

void LinAlg_add_multiple_of_row_to_row(int N, int M, double A[N][M], int row_start_index, int from_row, int to_row, double k)
{
    if(from_row < 0 || from_row >= N || to_row < 0 || to_row >= N){
        printf("Error: from_rom/to_row outside of matrix dimenions in add_multiple_of_row_to_row, exiting\n");
        return;
    }

    for(int i = row_start_index; i < M; i++){
        A[to_row][i] += A[from_row][i]*k;
    }
}

void LinAlg_matcopy(int N, int M, const double A[N][M], double B[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            B[i][j] = A[i][j];
        }
    }
}

void LinAlg_zeromat(int N, int M,double A[N][M]){
    for(int i = 0; i < N; i++){
        for(int j = 0; j < M; j++){
            A[i][j]= 0;
        }
    }
}

void LinAlg_eye(int N,double A[N][N])
{
    LinAlg_zeromat(N, N, A);
    for(int i = 0; i < N; i++){
        A[i][i]= 1;
    }
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

void LinAlg_printmat(int N, int M, const double A[N][M]){
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

void LinAlg_printvec(int N, double v[N]){
    for(int i = 0; i < N; i++){
        printf("[%.8f]\n",v[i]);
    }
    printf("\n");
}

void LinAlg_printvec_int(int N, int v[N]){
    for(int i = 0; i < N; i++){
        printf("[%d]\n",v[i]);
    }
    printf("\n");
}

