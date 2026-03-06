#ifndef OPTIMIZER_H
#define OPTIMIZER_H
#include "headers/linalg.h"
#include "headers/logging.h"

typedef struct{
    double x;
    double y;
    double z;
}Sample;

#define SAMPLE_COUNT 1000
#define PARAMETER_COUNT 9
extern Sample samples[SAMPLE_COUNT];

//According to my current unerstanding of my own implementation, the two functions above are the only functions strictly specific to the magnetometer cost function
//To generalize, they could be passed as function pointers to the optimizer below, this means ri and gradient_ri can be defined and declared in the sensor-specific file,
//while the rest of the optimizer can be in its own optimizer namespace
double optimizer_evaluate_ri(int param_count,
                             double opt_params[param_count],
                             double si[3]);

void optimizer_evaluate_gradient_ri(int param_count,
                                    double opt_params[param_count],
                                    double si[3],
                                    double grad_ri[param_count]);

void optimizer_evaluate_r_vec(int param_count,
                              int sample_count,
                              double opt_params[param_count],
                              const Sample* samples,
                              double r_vec[sample_count]);

void optimizer_evaluate_jacobian_r(int param_count,
                                   int sample_count,
                                   double opt_params[param_count],
                                   const Sample* samples,
                                   double J[sample_count][param_count]);

void optimizer_evaluate_gradient_r(int param_count,
                                   int sample_count,
                                   double g[param_count],
                                   double JT[param_count][sample_count],
                                   double r_vec[sample_count]); //gradient g = J^T*r

void optimizer_set_initial_guess_from_samples(int param_count,
                                              int sample_count,
                                              const Sample* samples,
                                              double theta_0[param_count]); //Computes rough guess of the optimum solution

void optimizer_LM_solver(double solution[PARAMETER_COUNT]);

#endif
