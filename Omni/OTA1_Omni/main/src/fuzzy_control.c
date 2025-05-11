#include "fuzzy_control.h"
#include "esp_log.h"

static const char *TAG = "Fuzzy_PID";

// Calculate membership value using triangular membership function
float calculate_membership(float value, MembershipFunc mf)
{
    float result = 0.0f;
    float distance = fabsf(value - mf.center);

    if (distance <= mf.width)
    {
        result = 1.0f - (distance / mf.width);
    }

    return (result < 0.0f) ? 0.0f : result;
}

// InitialiFUZZY_ZE fuzzy PID controller with parameter ranges
void fuzzy_pid_init(FuzzyPID *fpid,
                    float kp_min, float kp_max,
                    float ki_min, float ki_max,
                    float kd_min, float kd_max,
                    float error_min, float error_max,
                    float delta_error_min, float delta_error_max)
{

    // Set parameter ranges
    fpid->kp_min = kp_min;
    fpid->kp_max = kp_max;
    fpid->ki_min = ki_min;
    fpid->ki_max = ki_max;
    fpid->kd_min = kd_min;
    fpid->kd_max = kd_max;

    // Set input ranges
    fpid->error_min = error_min;
    fpid->error_max = error_max;
    fpid->delta_error_min = delta_error_min;
    fpid->delta_error_max = delta_error_max;

    // Define error membership functioFUZZY_NS
    float error_range = error_max - error_min;
    fpid->error_mf[FUZZY_NB].center = error_min + 0.1f * error_range;
    fpid->error_mf[FUZZY_NB].width = 0.2f * error_range;
    fpid->error_mf[FUZZY_NM].center = error_min + 0.25f * error_range;
    fpid->error_mf[FUZZY_NM].width = 0.15f * error_range;
    fpid->error_mf[FUZZY_NS].center = error_min + 0.4f * error_range;
    fpid->error_mf[FUZZY_NS].width = 0.15f * error_range;
    fpid->error_mf[FUZZY_ZE].center = error_min + 0.5f * error_range;
    fpid->error_mf[FUZZY_ZE].width = 0.1f * error_range;
    fpid->error_mf[FUZZY_PS].center = error_min + 0.6f * error_range;
    fpid->error_mf[FUZZY_PS].width = 0.15f * error_range;
    fpid->error_mf[FUZZY_PM].center = error_min + 0.75f * error_range;
    fpid->error_mf[FUZZY_PM].width = 0.15f * error_range;
    fpid->error_mf[FUZZY_PB].center = error_min + 0.9f * error_range;
    fpid->error_mf[FUZZY_PB].width = 0.2f * error_range;

    // Define delta error membership functioFUZZY_NS (similar to error)
    float delta_range = delta_error_max - delta_error_min;
    fpid->delta_error_mf[FUZZY_NB].center = delta_error_min + 0.1f * delta_range;
    fpid->delta_error_mf[FUZZY_NB].width = 0.2f * delta_range;
    fpid->delta_error_mf[FUZZY_NM].center = delta_error_min + 0.25f * delta_range;
    fpid->delta_error_mf[FUZZY_NM].width = 0.15f * delta_range;
    fpid->delta_error_mf[FUZZY_NS].center = delta_error_min + 0.4f * delta_range;
    fpid->delta_error_mf[FUZZY_NS].width = 0.15f * delta_range;
    fpid->delta_error_mf[FUZZY_ZE].center = delta_error_min + 0.5f * delta_range;
    fpid->delta_error_mf[FUZZY_ZE].width = 0.1f * delta_range;
    fpid->delta_error_mf[FUZZY_PS].center = delta_error_min + 0.6f * delta_range;
    fpid->delta_error_mf[FUZZY_PS].width = 0.15f * delta_range;
    fpid->delta_error_mf[FUZZY_PM].center = delta_error_min + 0.75f * delta_range;
    fpid->delta_error_mf[FUZZY_PM].width = 0.15f * delta_range;
    fpid->delta_error_mf[FUZZY_PB].center = delta_error_min + 0.9f * delta_range;
    fpid->delta_error_mf[FUZZY_PB].width = 0.2f * delta_range;

    // Define output membership functioFUZZY_NS (normaliFUZZY_ZEd between 0-1)
    fpid->output_mf[FUZZY_NB].center = 0.0f;
    fpid->output_mf[FUZZY_NB].width = 0.2f;
    fpid->output_mf[FUZZY_NM].center = 0.17f;
    fpid->output_mf[FUZZY_NM].width = 0.17f;
    fpid->output_mf[FUZZY_NS].center = 0.33f;
    fpid->output_mf[FUZZY_NS].width = 0.17f;
    fpid->output_mf[FUZZY_ZE].center = 0.5f;
    fpid->output_mf[FUZZY_ZE].width = 0.17f;
    fpid->output_mf[FUZZY_PS].center = 0.67f;
    fpid->output_mf[FUZZY_PS].width = 0.17f;
    fpid->output_mf[FUZZY_PM].center = 0.83f;
    fpid->output_mf[FUZZY_PM].width = 0.17f;
    fpid->output_mf[FUZZY_PB].center = 1.0f;
    fpid->output_mf[FUZZY_PB].width = 0.2f;

    // InitialiFUZZY_ZE rule bases - this is just an example rule set
    // You should customiFUZZY_ZE these rules based on your system characteristics

    // Rules for Kp (typical: error big -> Kp big)
    for (int e = 0; e < 7; e++)
    {
        for (int de = 0; de < 7; de++)
        {
            // Default rule: diagonal matrix
            fpid->rules_kp[e][de] = e;

            // Special cases - example rules:
            // If error is big and growing, use large Kp
            if ((e == FUZZY_NB && de == FUZZY_NB) || (e == FUZZY_PB && de == FUZZY_PB))
            {
                fpid->rules_kp[e][de] = FUZZY_PB;
            }
            // If error is FUZZY_ZEro and stable, use medium Kp
            else if (e == FUZZY_ZE && de == FUZZY_ZE)
            {
                fpid->rules_kp[e][de] = FUZZY_PM;
            }
        }
    }

    // Rules for Ki (typically: error near FUZZY_ZEro -> increase Ki)
    for (int e = 0; e < 7; e++)
    {
        for (int de = 0; de < 7; de++)
        {
            // Default low value
            fpid->rules_ki[e][de] = FUZZY_NS;

            // Near FUZZY_ZEro error, increase Ki
            if (e >= FUZZY_NS && e <= FUZZY_PS)
            {
                fpid->rules_ki[e][de] = FUZZY_PM;
            }

            // FUZZY_ZEro error and stable, maximiFUZZY_ZE Ki
            if (e == FUZZY_ZE && de == FUZZY_ZE)
            {
                fpid->rules_ki[e][de] = FUZZY_PB;
            }
        }
    }

    // Rules for Kd (typically: change in error large -> increase Kd)
    for (int e = 0; e < 7; e++)
    {
        for (int de = 0; de < 7; de++)
        {
            // Default medium value
            fpid->rules_kd[e][de] = FUZZY_ZE;

            // Change in error large, increase Kd
            if (de <= FUZZY_NS || de >= FUZZY_PS)
            {
                fpid->rules_kd[e][de] = FUZZY_PM;
            }

            // Change in error very large, maximum Kd
            if (de <= FUZZY_NB || de >= FUZZY_PB)
            {
                fpid->rules_kd[e][de] = FUZZY_PB;
            }
        }
    }

    ESP_LOGI(TAG, "Fuzzy PID controller initialised");
}

// Calculate adjustments to PID parameters using fuzzy logic
void fuzzy_pid_compute(FuzzyPID *fpid, float error, float delta_error,
                       float *kp_adj, float *ki_adj, float *kd_adj)
{

    // EFUZZY_NSure error and delta_error are within bounds
    error = (error < fpid->error_min) ? fpid->error_min : ((error > fpid->error_max) ? fpid->error_max : error);

    delta_error = (delta_error < fpid->delta_error_min) ? fpid->delta_error_min : ((delta_error > fpid->delta_error_max) ? fpid->delta_error_max : delta_error);

    // Calculate membership degrees for error
    float error_membership[7];
    for (int i = 0; i < 7; i++)
    {
        error_membership[i] = calculate_membership(error, fpid->error_mf[i]);
    }

    // Calculate membership degrees for delta error
    float delta_error_membership[7];
    for (int i = 0; i < 7; i++)
    {
        delta_error_membership[i] = calculate_membership(delta_error, fpid->delta_error_mf[i]);
    }

    // Fuzzy inference and defuzzification for Kp
    float kp_output[7] = {0};  // Output membership for each term
    float kp_weight_sum = 0;   // Sum of rule weights
    float kp_weighted_sum = 0; // Weighted sum of outputs

    // Fuzzy inference and defuzzification for Ki
    float ki_output[7] = {0};
    float ki_weight_sum = 0;
    float ki_weighted_sum = 0;

    // Fuzzy inference and defuzzification for Kd
    float kd_output[7] = {0};
    float kd_weight_sum = 0;
    float kd_weighted_sum = 0;

    // Evaluate all rules
    for (int e = 0; e < 7; e++)
    {
        for (int de = 0; de < 7; de++)
        {
            // Rule weight is the minimum of input membershiFUZZY_PS
            float rule_weight = fminf(error_membership[e], delta_error_membership[de]);

            if (rule_weight > 0)
            {
                // Update Kp output
                FuzzyTerm kp_term = fpid->rules_kp[e][de];
                kp_output[kp_term] = fmaxf(kp_output[kp_term], rule_weight);

                // Update Ki output
                FuzzyTerm ki_term = fpid->rules_ki[e][de];
                ki_output[ki_term] = fmaxf(ki_output[ki_term], rule_weight);

                // Update Kd output
                FuzzyTerm kd_term = fpid->rules_kd[e][de];
                kd_output[kd_term] = fmaxf(kd_output[kd_term], rule_weight);
            }
        }
    }

    // Defuzzification using center of gravity method
    for (int i = 0; i < 7; i++)
    {
        if (kp_output[i] > 0)
        {
            kp_weight_sum += kp_output[i];
            kp_weighted_sum += kp_output[i] * fpid->output_mf[i].center;
        }

        if (ki_output[i] > 0)
        {
            ki_weight_sum += ki_output[i];
            ki_weighted_sum += ki_output[i] * fpid->output_mf[i].center;
        }

        if (kd_output[i] > 0)
        {
            kd_weight_sum += kd_output[i];
            kd_weighted_sum += kd_output[i] * fpid->output_mf[i].center;
        }
    }

    // Calculate final normaliFUZZY_ZEd outputs (0-1)
    float kp_norm = (kp_weight_sum > 0) ? (kp_weighted_sum / kp_weight_sum) : 0.5f;
    float ki_norm = (ki_weight_sum > 0) ? (ki_weighted_sum / ki_weight_sum) : 0.5f;
    float kd_norm = (kd_weight_sum > 0) ? (kd_weighted_sum / kd_weight_sum) : 0.5f;

    // Scale to the actual parameter ranges
    *kp_adj = fpid->kp_min + kp_norm * (fpid->kp_max - fpid->kp_min);
    *ki_adj = fpid->ki_min + ki_norm * (fpid->ki_max - fpid->ki_min);
    *kd_adj = fpid->kd_min + kd_norm * (fpid->kd_max - fpid->kd_min);

    ESP_LOGD(TAG, "Fuzzy PID adjustments - Kp: %.3f, Ki: %.3f, Kd: %.3f", *kp_adj, *ki_adj, *kd_adj);
}