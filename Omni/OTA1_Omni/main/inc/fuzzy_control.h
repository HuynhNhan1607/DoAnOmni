#ifndef FUZZY_CONTROL_H
#define FUZZY_CONTROL_H

#include <stdio.h>
#include <math.h>

// Fuzzy linguistic variables
typedef enum
{
    FUZZY_NB, // Negative Big
    FUZZY_NM, // Negative Medium
    FUZZY_NS, // Negative Small
    FUZZY_ZE, // Zero
    FUZZY_PS, // Positive Small
    FUZZY_PM, // Positive Medium
    FUZZY_PB  // Positive Big
} FuzzyTerm;

// Membership function structure
typedef struct
{
    float center;
    float width;
} MembershipFunc;

// Fuzzy PID controller
typedef struct
{
    // Input ranges
    float error_min, error_max;
    float delta_error_min, delta_error_max;

    // Output ranges (adjustments to PID params)
    float kp_min, kp_max;
    float ki_min, ki_max;
    float kd_min, kd_max;

    // Membership functions for inputs
    MembershipFunc error_mf[7];       // NB, NM, NS, ZE, PS, PM, PB
    MembershipFunc delta_error_mf[7]; // NB, NM, NS, ZE, PS, PM, PB

    // Rule base for each parameter (Kp, Ki, Kd)
    FuzzyTerm rules_kp[7][7];
    FuzzyTerm rules_ki[7][7];
    FuzzyTerm rules_kd[7][7];

    // Output membership functions
    MembershipFunc output_mf[7]; // NB, NM, NS, ZE, PS, PM, PB
} FuzzyPID;

// Initialize a fuzzy PID controller
void fuzzy_pid_init(FuzzyPID *fpid,
                    float kp_min, float kp_max,
                    float ki_min, float ki_max,
                    float kd_min, float kd_max,
                    float error_min, float error_max,
                    float delta_error_min, float delta_error_max);

// Calculate adjustments to PID parameters using fuzzy logic
void fuzzy_pid_compute(FuzzyPID *fpid, float error, float delta_error,
                       float *kp_adj, float *ki_adj, float *kd_adj);

// Calculate membership value in a fuzzy set
float calculate_membership(float value, MembershipFunc mf);

#endif // FUZZY_CONTROL_H