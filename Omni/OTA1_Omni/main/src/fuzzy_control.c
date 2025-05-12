/*
 * fuzzy_pid.c – 7 × 7 Mamdani Fuzzy‑PID for omni‑wheel robot
 * Version: refactor with static const MF tables, NB/PB peak μ = 1
 * MIT License 2025
 */

#include <math.h>
#include <string.h>
#include "fuzzy_control.h" /* enum FuzzyTerm + struct declarations */
#include "esp_log.h"

static const char *TAG = "Fuzzy_PID";

/* ───────────────── 1. Static rule tables (49 luật) ───────────────── */
static const FuzzyTerm KP_RULES[7][7] = {
    {FUZZY_PB, FUZZY_PB, FUZZY_PM, FUZZY_PM, FUZZY_PS, FUZZY_ZE, FUZZY_ZE},
    {FUZZY_PB, FUZZY_PM, FUZZY_PM, FUZZY_PS, FUZZY_ZE, FUZZY_ZE, FUZZY_NS},
    {FUZZY_PM, FUZZY_PM, FUZZY_PS, FUZZY_ZE, FUZZY_NS, FUZZY_NS, FUZZY_NM},
    {FUZZY_PM, FUZZY_PS, FUZZY_ZE, FUZZY_ZE, FUZZY_ZE, FUZZY_NS, FUZZY_NM},
    {FUZZY_PS, FUZZY_ZE, FUZZY_NS, FUZZY_ZE, FUZZY_ZE, FUZZY_NM, FUZZY_NM},
    {FUZZY_ZE, FUZZY_NS, FUZZY_NS, FUZZY_NS, FUZZY_NM, FUZZY_NM, FUZZY_NB},
    {FUZZY_ZE, FUZZY_ZE, FUZZY_NM, FUZZY_NM, FUZZY_NM, FUZZY_NB, FUZZY_NB}};

static const FuzzyTerm KI_RULES[7][7] = {
    {FUZZY_NS, FUZZY_NS, FUZZY_ZE, FUZZY_NS, FUZZY_NS, FUZZY_NM, FUZZY_NB},
    {FUZZY_NS, FUZZY_ZE, FUZZY_PS, FUZZY_PS, FUZZY_ZE, FUZZY_NM, FUZZY_NB},
    {FUZZY_ZE, FUZZY_PS, FUZZY_PM, FUZZY_PM, FUZZY_PS, FUZZY_NS, FUZZY_NM},
    {FUZZY_PS, FUZZY_PM, FUZZY_PB, FUZZY_PB, FUZZY_PM, FUZZY_PS, FUZZY_NS},
    {FUZZY_PS, FUZZY_PM, FUZZY_PM, FUZZY_PM, FUZZY_PM, FUZZY_PS, FUZZY_NS},
    {FUZZY_NS, FUZZY_NS, FUZZY_NS, FUZZY_NS, FUZZY_NS, FUZZY_NM, FUZZY_NB},
    {FUZZY_NS, FUZZY_NS, FUZZY_NS, FUZZY_NS, FUZZY_NS, FUZZY_NB, FUZZY_NB}};

static const FuzzyTerm KD_RULES[7][7] = {
    {FUZZY_PB, FUZZY_PM, FUZZY_PM, FUZZY_PS, FUZZY_ZE, FUZZY_ZE, FUZZY_ZE},
    {FUZZY_PM, FUZZY_PM, FUZZY_PS, FUZZY_ZE, FUZZY_ZE, FUZZY_NS, FUZZY_NS},
    {FUZZY_PM, FUZZY_PS, FUZZY_ZE, FUZZY_NS, FUZZY_NS, FUZZY_NM, FUZZY_NM},
    {FUZZY_PS, FUZZY_ZE, FUZZY_NS, FUZZY_NS, FUZZY_NS, FUZZY_NM, FUZZY_NM},
    {FUZZY_ZE, FUZZY_ZE, FUZZY_NS, FUZZY_NS, FUZZY_NS, FUZZY_NM, FUZZY_PM},
    {FUZZY_ZE, FUZZY_NS, FUZZY_NM, FUZZY_NM, FUZZY_NM, FUZZY_PM, FUZZY_PB},
    {FUZZY_ZE, FUZZY_NS, FUZZY_NM, FUZZY_NM, FUZZY_PM, FUZZY_PB, FUZZY_PB}};

/* ───────────────── 2. Static MF tables (peaks + half‑base) ───────────────── */
/* 50 % overlap: centre spacing = width = 1⁄6 (≈0.1667) */
static const float ERR_POS[7] = {0.0f, 0.1667f, 0.3333f, 0.5f, 0.6667f, 0.8333f, 1.0f};
static const float ERR_WID[7] = {0.1667f, 0.1667f, 0.1667f, 0.1667f, 0.1667f, 0.1667f, 0.1667f};

static const float OUT_POS[7] = {0.0f, 0.1667f, 0.3333f, 0.5f, 0.6667f, 0.8333f, 1.0f};
static const float OUT_WID[7] = {0.17f, 0.17f, 0.17f, 0.17f, 0.17f, 0.17f, 0.17f};

/* ───────────────── 3. Helper – triangular μ(x) ───────────────── */
static inline float tri_mu(float x, MembershipFunc mf)
{
    float d = fabsf(x - mf.center);
    return (d >= mf.width) ? 0.0f : 1.0f - d / mf.width;
}

/* ───────────────── 4. Init ───────────────── */
void fuzzy_pid_init(FuzzyPID *fpid,
                    float kp_min, float kp_max,
                    float ki_min, float ki_max,
                    float kd_min, float kd_max,
                    float e_min, float e_max,
                    float ec_min, float ec_max)
{
    /* store ranges */
    fpid->kp_min = kp_min;
    fpid->kp_max = kp_max;
    fpid->ki_min = ki_min;
    fpid->ki_max = ki_max;
    fpid->kd_min = kd_min;
    fpid->kd_max = kd_max;

    fpid->error_min = e_min;
    fpid->error_max = e_max;
    fpid->delta_error_min = ec_min;
    fpid->delta_error_max = ec_max;

    const float e_range = e_max - e_min;
    const float ec_range = ec_max - ec_min;

    for (int i = 0; i < 7; ++i)
    {
        /* error & delta‑error MF */
        fpid->error_mf[i].center = e_min + ERR_POS[i] * e_range;
        fpid->error_mf[i].width = ERR_WID[i] * e_range;
        fpid->delta_error_mf[i].center = ec_min + ERR_POS[i] * ec_range;
        fpid->delta_error_mf[i].width = ERR_WID[i] * ec_range;

        /* output MF (0‒1 domain) */
        fpid->output_mf[i].center = OUT_POS[i];
        fpid->output_mf[i].width = OUT_WID[i];
    }

    /* copy rule bases */
    memcpy(fpid->rules_kp, KP_RULES, sizeof(KP_RULES));
    memcpy(fpid->rules_ki, KI_RULES, sizeof(KI_RULES));
    memcpy(fpid->rules_kd, KD_RULES, sizeof(KD_RULES));

    ESP_LOGI(TAG, "Fuzzy PID initialised (static MF, 49‑rule)");
}

/* ───────────────── 5. Compute ───────────────── */
void fuzzy_pid_compute(FuzzyPID *fpid, float e_raw, float ec_raw,
                       float *kp_adj, float *ki_adj, float *kd_adj)
{
    /* clamp */
    if (e_raw < fpid->error_min)
        e_raw = fpid->error_min;
    else if (e_raw > fpid->error_max)
        e_raw = fpid->error_max;
    if (ec_raw < fpid->delta_error_min)
        ec_raw = fpid->delta_error_min;
    else if (ec_raw > fpid->delta_error_max)
        ec_raw = fpid->delta_error_max;

    /* fuzzification */
    float mu_e[7];
    float mu_ec[7];
    for (int i = 0; i < 7; ++i)
    {
        mu_e[i] = tri_mu(e_raw, fpid->error_mf[i]);
        mu_ec[i] = tri_mu(ec_raw, fpid->delta_error_mf[i]);
    }

    /* inference (max‑min) */
    float agg_kp[7] = {0}, agg_ki[7] = {0}, agg_kd[7] = {0};
    for (int i = 0; i < 7; ++i)
        for (int j = 0; j < 7; ++j)
        {
            float w = (mu_e[i] < mu_ec[j]) ? mu_e[i] : mu_ec[j];
            if (w == 0)
                continue;
            FuzzyTerm tkp = fpid->rules_kp[i][j];
            FuzzyTerm tki = fpid->rules_ki[i][j];
            FuzzyTerm tkd = fpid->rules_kd[i][j];
            if (w > agg_kp[tkp])
                agg_kp[tkp] = w;
            if (w > agg_ki[tki])
                agg_ki[tki] = w;
            if (w > agg_kd[tkd])
                agg_kd[tkd] = w;
        }

    /* defuzz */
    float sum_kp = 0, cog_kp = 0;
    float sum_ki = 0, cog_ki = 0;
    float sum_kd = 0, cog_kd = 0;
    for (int i = 0; i < 7; ++i)
    {
        if (agg_kp[i])
        {
            sum_kp += agg_kp[i];
            cog_kp += agg_kp[i] * fpid->output_mf[i].center;
        }
        if (agg_ki[i])
        {
            sum_ki += agg_ki[i];
            cog_ki += agg_ki[i] * fpid->output_mf[i].center;
        }
        if (agg_kd[i])
        {
            sum_kd += agg_kd[i];
            cog_kd += agg_kd[i] * fpid->output_mf[i].center;
        }
    }
    float n_kp = sum_kp ? cog_kp / sum_kp : 0.5f;
    float n_ki = sum_ki ? cog_ki / sum_ki : 0.5f;
    float n_kd = sum_kd ? cog_kd / sum_kd : 0.5f;

    /* scale to real gains */
    *kp_adj = fpid->kp_min + n_kp * (fpid->kp_max - fpid->kp_min);
    *ki_adj = fpid->ki_min + n_ki * (fpid->ki_max - fpid->ki_min);
    *kd_adj = fpid->kd_min + n_kd * (fpid->kd_max - fpid->kd_min);

    ESP_LOGD(TAG, "Fuzzy PID: Kp=%.3f Ki=%.3f Kd=%.3f", *kp_adj, *ki_adj, *kd_adj);
}
