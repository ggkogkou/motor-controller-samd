#pragma once

#include <cmath>
#include "svpwm.hpp"

extern void pwm_set_duties(float da, float db, float dc);
extern void wait_pwm_tick();

struct OpenLoopParams {
    float vdc;            // [V] DC bus (measure once)
    float Ts;             // [s] control period (1/PWM freq)
    float vmag_frac;      // pu of Vdc for vector magnitude (e.g. 0.06f)
    float align_time_s;   // [s] alignment duration (e.g. 0.5f)
    float omega_target_e; // [rad/s] electrical target speed (e.g. 60.0f ~ 9.55Hz)
    float accel_e;        // [rad/s^2] electrical acceleration (e.g. 50.0f)
};

inline void spin_open_loop(const OpenLoopParams& p) {
    using namespace SpaceVectorModulation;

    // Instantiate with your current ZSM (MIDPOINT_CLAMP in your code)
    SVPWM sv(p.vdc, ZeroSequenceModulationType::MIDPOINT_CLAMP);

    const float Vmag = p.vmag_frac * p.vdc;     // absolute volts for the vector
    float theta = 0.0f;                         // electrical angle
    float omega = 0.0f;                         // electrical speed command

    // --- 1) Alignment: hold vector at theta = 0
    int N_align = (int)(p.align_time_s / p.Ts);
    for (int k = 0; k < N_align; ++k) {
        auto d = sv.compute(Vmag, 0.0f);        // cos=1, sin=0
        pwm_set_duties(d.dutyCycleA, d.dutyCycleB, d.dutyCycleC);
        wait_pwm_tick();
    }

    // --- 2) Ramp speed and spin with constant magnitude
    const float sign = (p.omega_target_e >= 0.0f) ? 1.0f : -1.0f;
    while ((sign > 0.0f && omega < p.omega_target_e) ||
           (sign < 0.0f && omega > p.omega_target_e)) {

        omega += sign * p.accel_e * p.Ts;
        theta += omega * p.Ts;

        // wrap angle
        if (theta >= 2.0f * M_PI)
            theta -= 2.0f * M_PI;
        else if (theta < 0.0f)
            theta += 2.0f * M_PI;

        float vAlpha = Vmag * std::cos(theta);
        float vBeta  = Vmag * std::sin(theta);

        auto d = sv.compute(vAlpha, vBeta);
        pwm_set_duties(d.dutyCycleA, d.dutyCycleB, d.dutyCycleC);
        wait_pwm_tick();
    }

}
