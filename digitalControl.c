//------------------------------------------------------------------------------
// LIBRARIES
//------------------------------------------------------------------------------

#include "digitalControl.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

//------------------------------------------------------------------------------
// DEFINITIONSS
//------------------------------------------------------------------------------

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//------------------------------------------------------------------------------
// FUNCTIONS
//------------------------------------------------------------------------------

static float square(float x) {
    return x * x;
}

//------------------------------------------------------------------------------
void saturatorSet(Saturator *this, const float max, const float min) {
    this->max = max;
    this->min = min;
}

float applySaturator(Saturator *this, const float value) {
    if (value > this->max) {
        return this->max;
    }
    if (value < this->min) {
        return this->min;
    }
    return value;
}

//------------------------------------------------------------------------------
void rampInit(RampGenerator *this, const float samplingTime) {
    this->samplingTime = samplingTime;
    rampReset(this);
}

void rampReset(RampGenerator *this) {
    this->upCounter = 0;
    this->upTime = 0;
    this->finalValue = 0.0f;
    this->initialValue = 0.0f;
    this->previousOutput = 0.0f;
    this->deltaTime = 0.0f;
}

void rampSet(RampGenerator *this, const float upTime, const float finalValue) {
    this->upTime = upTime;
    this->finalValue = finalValue;
    this->initialValue = this->previousOutput;
    this->upCounter = roundf(this->upTime / this->samplingTime);
    this->deltaTime = (this->samplingTime / this->upTime) * (this->finalValue - this->initialValue);
}

float calcRamp(RampGenerator *this) {
    if (this->upCounter) {
        this->previousOutput = this->finalValue - this->upCounter * this->deltaTime;
        this->upCounter--;
    } else {
        this->previousOutput = this->finalValue;
    }
    return this->previousOutput;
}

//------------------------------------------------------------------------------
void integratorInit(Integrator *this, const float samplingTime) {
    this->samplingTime = samplingTime;
    integratorReset(this);
}

void integratorReset(Integrator *this) {
    this->previousInput = 0.0f;
    this->previousOutput = 0.0f;
}

// Integrates using Tustin discretization method
float integrate(Integrator *this, const float input) {
    this->previousOutput += (this->samplingTime / 2.0) * (input + this->previousInput);
    this->previousInput = input;
    return this->previousOutput;
}

//------------------------------------------------------------------------------
void diffInit(Differentiator *this, const float samplingTime) {
    this->samplingTime = samplingTime;
    diffReset(this);
}

void diffReset(Differentiator *this) {
    this->previousInput = 0.0f;
}

// Differentiates using Backward discretization method
float differentiate(Differentiator *this, const float input) {
    float output = (input - this->previousInput) / this->samplingTime;
    this->previousInput = input;
    return output;
}

//------------------------------------------------------------------------------
void filterInit(FirstOrderFilter *this, const float zero, const float pole, const float samplingTime) {
    this->samplingTime = samplingTime;
    filterReset(this);
    filterDiscreet(this, zero, pole);
}

void filterReset(FirstOrderFilter *this) {
    this->previousInput = 0.0f;
    this->previousOutput = 0.0f;
}

// Determines the filter coefficients using the Tustin discretization method
void filterDiscreet(FirstOrderFilter *this, const float zero, const float pole) {
    this->coef[0] = (2.0 - pole * this->samplingTime) / (2.0 + pole * this->samplingTime);
    this->coef[1] = ((2.0 * (pole / zero) + pole * this->samplingTime) / (2.0 + pole * this->samplingTime));
    this->coef[2] = ((-2.0 * (pole / zero) + pole * this->samplingTime) / (2.0 + pole * this->samplingTime));
}

float filterProcess(FirstOrderFilter *this, const float input) {
    this->previousOutput = this->coef[0] * this->previousOutput + this->coef[1] * input + this->coef[2] * this->previousInput;
    this->previousInput = input;
    return this->previousOutput;
}

//------------------------------------------------------------------------------
/* This is meant to be the minimalistic implementation
 * of a first order filter, there is only one pole and
 * it uses the simpler forward discretization method
 */
void sFilterInit(SimplifiedFirstOrderFilter *this, float pole, const float samplingTime) {
    this->samplingTime = samplingTime;
    sFilterReset(this);
    sFilterDiscreet(this, pole);
}

void sFilterReset(SimplifiedFirstOrderFilter *this) {
    this->previousOutput = 0.0f;
}

// Determines the filter coefficients using the Forward discretization method
void sFilterDiscreet(SimplifiedFirstOrderFilter *this, const float pole) {
    this->coef[1] = pole * this->samplingTime;
    this->coef[0] = 1.0 - this->coef[1];
}

float sFilterProcess(SimplifiedFirstOrderFilter *this, const float input) {
    this->previousOutput = this->coef[0] * this->previousOutput + this->coef[1] * input;
    return this->previousOutput;
}

//------------------------------------------------------------------------------
// Initializes PI controller
void piInit(PI *this, const float samplingTime, const bool useSaturator) {
    this->samplingTime = samplingTime;
    this->useSaturator = useSaturator;
    piReset(this);
}

void piReset(PI *this) {
    this->controlSignal = 0.0f;
    this->previousError = 0.0f;
}

// Determines the discrete gains using the Tustin discretization method
void piDiscreet(PI *this, const float Kp, const float Ti) {
    this->Kc = Kp * (2.0 + this->samplingTime / Ti) / (2.0);
    this->zc = (2.0 - this->samplingTime / Ti) / (2.0 + this->samplingTime / Ti);
}

// Design controller using pole zero cancellation
void piPoleZeroCancelationProject(PI *this, const float wcl, const float Kol, const float wol) {
    const float Ti = 1.0 / wol;
    const float Kp = (wcl * Ti) / Kol;
    piDiscreet(this, Kp, Ti);
}

// Design controller for desired closed loop characteristics
void piClosedLoopResponseProject(PI *this, const float Mov, const float ts2, const float Kol, const float wol) {
    // Desired closed loop characteristics
    const float xi = fabsf(logf(Mov / 100.0f) / sqrtf(square(logf(Mov / 100.0f)) + square(M_PI)));
    const float wn = 4.0 / (xi * ts2);
    // Calculate the analog gains
    const float Kp = (2.0 * xi * wn - wol) / (Kol * wol);
    const float Ti = (2.0 * xi * wn - wol) / (square(wn));
    piDiscreet(this, Kp, Ti);
}

// Calculate control output
float piControl(PI *this, const float setPoint, const float feedBack) {
    float error = setPoint - feedBack;
    this->controlSignal = this->controlSignal + this->Kc * (error - this->zc * this->previousError);
    this->previousError = error;
    if (this->useSaturator) {
        this->controlSignal = applySaturator(&this->sat, this->controlSignal);
    }
    return this->controlSignal;
}

//------------------------------------------------------------------------------
// Initializes PI_D controller
void pidInit(PI_D *this, const float samplingTime, const bool useSaturator, const bool usePreFilter) {
    this->samplingTime = samplingTime;
    this->useSaturator = useSaturator;
    this->usePreFilter = usePreFilter;
    pidReset(this);
}

void pidReset(PI_D *this) {
    for (uint16_t i = 0; i < 3; i++) {
        this->feedBack[i] = 0.0f;
        this->error[i] = 0.0f;
        this->controlSignal[i] = 0.0f;
    }
    this->spFiltered = 0.0f;
    filterReset(&this->preFilter);
}

// Determines the discrete gains using the Tustin discretization method
void pidDiscreet(PI_D *this, const float Kp, const float Ti, const float Td, const float N) {
    this->coef[0] = (this->samplingTime + 2.0 * Td / N) / (this->samplingTime + Td / N);
    this->coef[1] = -(Td / N) / (this->samplingTime + Td / N);
    this->coef[2] = Kp * (2.0 * Ti + this->samplingTime) / (2.0 * Ti);
    this->coef[3] = -Kp * (2.0 * Ti * this->samplingTime - square(this->samplingTime) + 4.0 * Ti * Td / N) / (2.0 * Ti * (this->samplingTime + Td / N));
    this->coef[4] = Kp * ((2.0 * Ti - this->samplingTime) / (2.0 * Ti)) * ((Td / N) / (this->samplingTime + Td / N));
    this->coef[5] = -Kp * (Td / (this->samplingTime + Td / N));
    this->coef[6] = -2.0 * this->coef[5];
    this->coef[7] = this->coef[5];
}

// Design controller for desired closed loop characteristics
void pidClosedLoopResponseProject(PI_D *this, const float Mov, const float ts2, const float Kol, const float wol) {
    //  Derivative filter coefficient
    const float N = 10.0f;
    // Desired closed loop characteristics
    const float xi = fabsf(logf(Mov / 100.0f) / sqrtf(square(logf(Mov / 100.0f)) + square(M_PI)));
    const float wn = 4.0 / (xi * ts2);
    const float real = fabsf(xi * wn);
    // const float imag = fabsf(wn * sqrtf(1 - square(xi)));
    // Determines the closed-loop zero location
    const float minZ = wn / (2.0 * xi);
    const float maxZ = square(wn) / (2.0 * xi * wn - wol);
    const float zero = sqrtf(minZ * maxZ);
    // Calculate the analog gains
    const float Ti = 1.0 / zero;
    const float Kp = wn / (Kol * (2.0 * xi * zero - wn));
    const float Td = zero / square(wn) - 1.0 / (Kp * Kol * wol);
    pidDiscreet(this, Kp, Ti, Td, N);
    // Project the pre filter
    if (this->usePreFilter) {
        filterInit(&this->preFilter, 10.0f * real, zero, this->samplingTime);
    }
}

// Calculate the control output
float pidControl(PI_D *this, const float setPoint, const float feedBack) {
    this->feedBack[2] = feedBack;
    if (this->usePreFilter) {
        this->spFiltered = filterProcess(&this->preFilter, setPoint);
    } else {
        this->spFiltered = setPoint;
    }
    this->error[2] = this->spFiltered - feedBack;
    // Control signal
    this->controlSignal[2] = this->coef[0] * this->controlSignal[1] + this->coef[1] * this->controlSignal[0];
    for (uint16_t i = 0; i < 3; i++) {
        this->controlSignal[2] += this->coef[7 - i] * this->feedBack[i];
        this->controlSignal[2] += this->coef[4 - i] * this->error[i];
    }
    if (this->useSaturator) {
        this->controlSignal[2] = applySaturator(&this->sat, this->controlSignal[2]);
    }
    // Apply delay to internal buffers
    for (uint16_t i = 0; i < 2; i++) {
        this->feedBack[i] = this->feedBack[i + 1];
        this->error[i] = this->error[i + 1];
        this->controlSignal[i] = this->controlSignal[i + 1];
    }
    return this->controlSignal[2];
}

//------------------------------------------------------------------------------
void observerInit(FirstOrderObserver *this, const float samplingTime) {
    this->samplingTime = samplingTime;
    observerReset(this);
}

void observerReset(FirstOrderObserver *this) {
    this->previousInput = 0.0f;
    this->previousOutput = 0.0f;
}

// FirstOrderObserver design (ZOH approach)
void observerProject(FirstOrderObserver *this, const float Kol, const float wol) {
    this->zd = expf(-this->samplingTime * wol);
    this->Kd = Kol * (1 - this->zd);
}

float observerProcess(FirstOrderObserver *this, const float input) {
    this->previousOutput = this->zd * this->previousOutput + this->Kd * (this->previousInput);
    this->previousInput = input;
    return this->previousOutput;
}

//------------------------------------------------------------------------------
void ifocInit(IFOC *this, const float samplingTime, const bool useSaturator, const bool useFeedforward) {
    piInit(&this->piIqs, samplingTime, useSaturator);
    piInit(&this->piIds, samplingTime, useSaturator);
    piInit(&this->piIdm, samplingTime, useSaturator);
    observerInit(&this->IdmObserver, samplingTime);
    this->samplingTime = samplingTime;
    this->useFeedforward = useFeedforward;
    this->theta = -M_PI / 2.0;
}

void ifocReset(IFOC *this) {
    piReset(&this->piIqs);
    piReset(&this->piIds);
    piReset(&this->piIdm);
    observerReset(&this->IdmObserver);
    this->theta = -M_PI / 2.0;
}

void ifocSetSaturators(IFOC *this, const float max_voltage, const float max_ids, const float min_ids) {
    saturatorSet(&this->piIqs.sat, max_voltage, -max_voltage);
    saturatorSet(&this->piIds.sat, max_voltage, -max_voltage);
    saturatorSet(&this->piIdm.sat, max_ids, -min_ids);
}

void ifocProject(IFOC *this, const uint16_t p, const float fn, const float rs, const float rr, const float Xls, const float Xlr, const float Xm) {
    const float wn = 2 * M_PI * fn;
    // Reactances
    const float Xss = Xm + Xls;
    const float Xrr = Xm + Xlr;
    // Inductances
    const float Lm = Xm / wn;
    const float Lss = Xss / wn;
    const float Lrr = Xrr / wn;
    // Secondary coefficients
    const float sigma = 1 - powf(Lm, 2) / (Lss * Lrr);
    this->delta = powf(Lm, 2) / (sigma * Lss * Lrr);
    this->Lsigmas = sigma * Lss;
    this->eta = rr / Lrr;
    this->gamma = (rs + powf(Lm / Lrr, 2) * rr) / this->Lsigmas;
    // Torque-current proportionality constant
    this->Km = (3 * ((float)p) / 4) * (powf(Lm, 2) / Lrr);
    // Project the observer
    observerProject(&this->IdmObserver, 1.0, this->eta);
    // Project the controllers (3 times faster than in open-loop)
    piPoleZeroCancelationProject(&this->piIqs, 3.0 * this->gamma, 1.0 / (this->gamma * this->Lsigmas), this->gamma);
    piPoleZeroCancelationProject(&this->piIds, 3.0 * this->gamma, 1.0 / (this->gamma * this->Lsigmas), this->gamma);
    piPoleZeroCancelationProject(&this->piIdm, 3.0 * this->eta, 1.0, this->eta);
}

void ifocControl(IFOC *this, const float spTe, const float spIdm, const float currentA, const float currentB, const float wr, float *const uBeta, float *const uAlpha) {
    const float ct = cosf(this->theta);
    const float st = sinf(this->theta);
    // Alpha-Beta Transform
    const float iBeta = currentA;
    const float iAlpha = (-currentA - 2.0 * currentB) / sqrtf(3.0);
    // QD0 Transform
    const float iqs = ct * iBeta - st * iAlpha;
    const float ids = st * iBeta + ct * iAlpha;
    // Magnetizing current
    const float idm = observerProcess(&this->IdmObserver, ids);
    // Referential speed
    const float wref = wr + ((fabs(idm) > EPSILON) ? ((this->eta * iqs) / idm) : 0.0);
    // Iqs setpoint
    const float spIqs = spTe / (this->Km * spIdm);
    // Magnetizing current controller
    const float spIds = piControl(&this->piIdm, spIdm, idm);
    // Current controllers
    float eqs = 0.0f, eds = 0.0f;
    if (this->useFeedforward) {
        eqs = this->Lsigmas * (wref * ids + this->delta * wr * idm);
        eds = -this->Lsigmas * (wref * iqs + this->delta * this->eta * idm);
    }
    float uqs = eqs + piControl(&this->piIqs, spIqs, iqs);
    float uds = eds + piControl(&this->piIds, spIds, ids);
    // QD0 Transform
    if (uBeta != NULL) {
        *uBeta = ct * uqs + st * uds;
    }
    if (uAlpha != NULL) {
        *uAlpha = -st * uqs + ct * uds;
    }
    // Integrates the referential angle
    this->theta += wref * this->samplingTime;
    if (this->theta >= 2.0 * M_PI) {
        this->theta -= 2.0 * M_PI;
    } else if (this->theta <= 0) {
        this->theta += 2.0 * M_PI;
    }
}

//------------------------------------------------------------------------------
// END
//------------------------------------------------------------------------------
