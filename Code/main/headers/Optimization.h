#ifndef OPTIMIZATION_H
#define OPTIMIZATION_H
#include "headers/LinAlg.h"

typedef struct{
    double x;
    double y;
    double z;
}Sample;

#define SAMPLE_COUNT 1000
#define PARAMETER_COUNT 9
extern Sample Samples[SAMPLE_COUNT];

//According to my current unerstanding of my own implementation, the two functions above are the only functions strictly specific to the magnetometer cost function
//To generalize, they could be passed as function pointers to the optimizer below, this means ri and gradient_ri can be defined and declared in the sensor-specific file,
//while the rest of the optimizer can be in its own optimizer namespace.
double Namespace_evaluate_ri(int param_count, double opt_params[param_count], double si[3]);

void Namespace_evaluate_gradient_ri(int param_count, double opt_params[param_count], double si[3], double grad_ri[param_count]);

void Namespace_evaluate_r_vec(int param_count, int sample_count, double opt_params[param_count], const Sample* samples, double r_vec[sample_count]);

void Namespace_evaluate_jacobian_r(int param_count, int sample_count, double opt_params[param_count], const Sample* samples, double J[sample_count][param_count]);

void Namespace_evaluate_gradient_r(int param_count, int sample_count, double g[param_count], double JT[param_count][sample_count], double r_vec[sample_count]); //gradient g = J^T*r

void Namespace_set_initial_guess_from_samples(int param_count, int sample_count, const Sample* samples, double theta_0[param_count]); //Computes rough guess of the optimum solution

void Namespace_LM_solver();

void Namespace_get_magnetometer_calib(double theta[9], double correction_matrix[3][3], double correction_vector[3]);

void Namespace_get_corrected_mag_vector(double correction_matrix[3][3], double correction_vector[3], Sample si, double m_corr[3]);

#endif
