#include "headers/optimizer.h"
#include "headers/ValidationData.h"

//Large variables for LM-solver
//Total RAM required from this is approx 200kB, well within RP2350 RAM of 512kB, but optimizations can be done
double theta[PARAMETER_COUNT];
double d_theta[PARAMETER_COUNT]; //72B, parameter increment
double J[SAMPLE_COUNT][PARAMETER_COUNT]; //Jacobian, 72kB
double JT[PARAMETER_COUNT][SAMPLE_COUNT]; //Jacobian transposed, 72kB
double JTJ[PARAMETER_COUNT][PARAMETER_COUNT]; //648B
double residue_vec[SAMPLE_COUNT]; //8kB
double gradient_vec[PARAMETER_COUNT];
Sample samples[SAMPLE_COUNT];//24kB

double optimizer_evaluate_ri(int param_count,
                             double opt_params[param_count],
                             double si[3])
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

void optimizer_evaluate_gradient_ri(int param_count,
                                    double opt_params[param_count],
                                    double si[3],
                                    double grad_ri[param_count])
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

void optimizer_evaluate_r_vec(int param_count,
                              int sample_count,
                              double opt_params[param_count],
                              const Sample* samples,
                              double r_vec[sample_count])
{
    double si[3];
    for(int i = 0; i < sample_count; i++){
        si[0] = samples[i].x;
        si[1] = samples[i].y;
        si[2] = samples[i].z;
        r_vec[i] = optimizer_evaluate_ri(param_count, opt_params, si);
    }
}

void optimizer_evaluate_jacobian_r(int param_count,
                                   int sample_count,
                                   double opt_params[param_count],
                                   const Sample* samples,
                                   double J[sample_count][param_count])
{
    //The jacobian of r_vec is a matrix where each row is a transposed gradient of ri
    double grad_ri[9];
    double si[3];
    for(int i = 0; i < sample_count; i++){
        si[0] = samples[i].x;
        si[1] = samples[i].y;
        si[2] = samples[i].z;
        optimizer_evaluate_gradient_ri(param_count, opt_params, si, grad_ri);
        for(int j = 0; j < param_count; j++){
            J[i][j] = grad_ri[j];
        }
    }
}

void optimizer_evaluate_gradient_r(int param_count,
                                   int sample_count,
                                   double g[param_count],
                                   double JT[param_count][sample_count],
                                   double r_vec[sample_count])
{
    //g = JT*r
    linalg_zerovec(param_count,g);
    for(int i = 0; i < param_count; i++){
        for(int j = 0; j < sample_count; j++){
            g[i] += JT[i][j]*r_vec[j];
        }
    }
}

void optimizer_set_initial_guess_from_samples(int param_count,
                                              int sample_count,
                                              const Sample* samples,
                                              double theta_0[param_count])
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

void optimizer_LM_solver(double solution[PARAMETER_COUNT])
{
    optimizer_set_initial_guess_from_samples(PARAMETER_COUNT, SAMPLE_COUNT,samples,theta);
    double lambda = 1E-4; //This worked in matlab
    double lambdaEye[PARAMETER_COUNT][PARAMETER_COUNT];
    double JTJ_plus_lambda_eye[PARAMETER_COUNT][PARAMETER_COUNT];
    int iter = 0;
    int max_iter = 1000;

    do{
        optimizer_evaluate_r_vec(PARAMETER_COUNT,SAMPLE_COUNT, theta,samples,residue_vec);
        optimizer_evaluate_jacobian_r(PARAMETER_COUNT,SAMPLE_COUNT,theta,samples,J);
        linalg_mattranspose(SAMPLE_COUNT,PARAMETER_COUNT,J,JT);
        optimizer_evaluate_gradient_r(PARAMETER_COUNT,SAMPLE_COUNT,gradient_vec,JT,residue_vec);
        linalg_matmatmul_no_alias(PARAMETER_COUNT,SAMPLE_COUNT,JT,J,JTJ);
        linalg_eye(PARAMETER_COUNT,lambdaEye);
        linalg_matscalmult(PARAMETER_COUNT,PARAMETER_COUNT,lambdaEye,lambda,lambdaEye);
        linalg_matmatadd(PARAMETER_COUNT,PARAMETER_COUNT,JTJ,lambdaEye,JTJ_plus_lambda_eye); //JT*J + lambda*I
        //update theta:
        linalg_vecscalmult(PARAMETER_COUNT,gradient_vec,gradient_vec, -1.0);
        linalg_solve_linear_system_square_in_place(PARAMETER_COUNT,JTJ_plus_lambda_eye,d_theta,gradient_vec);
        linalg_vecvecadd(PARAMETER_COUNT,d_theta,theta,theta);
        iter++;
    } while (linalg_vecnorm(PARAMETER_COUNT,d_theta) > 1E-6 && iter < max_iter);
    
    if(iter == max_iter){
        PRINTNUM("Maximum allowed iterations of %d exceeded, some problem occured, probably bad initial estimate!\n", max_iter);
    }else{
        PRINTNUM("Solution to the system at %d iterations with vecnorm(d_theta) = ", iter);
        PRINTNUM("%f\n", linalg_vecnorm(PARAMETER_COUNT,d_theta));
        linalg_printvec(PARAMETER_COUNT, theta);
        linalg_veccopy(PARAMETER_COUNT,theta,solution);
    }
}
