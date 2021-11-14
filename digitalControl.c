//------------------------------------------------------------------------------
// LIBRARIES
//------------------------------------------------------------------------------

#include "digitalControl.h"
#include <math.h>
#include <stdint.h>

//------------------------------------------------------------------------------
// DEFINITIONSS
//------------------------------------------------------------------------------

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

//------------------------------------------------------------------------------
// FUNCTIONS
//------------------------------------------------------------------------------

static float square(float x)
{
    return x * x;
}

//------------------------------------------------------------------------------
void saturatorSet(Saturator *this, const float max, const float min)
{
    this->max = max;
    this->min = min;
}

float appplySaturator(Saturator *this, const float value)
{
    if (value > this->max)
    {
        return this->max;
    }
    if (value < this->min)
    {
        return this->min;
    }
    return value;
}

//------------------------------------------------------------------------------
void rampInit(RampGenerator *this, const float samplingTime)
{
    this->samplingTime = samplingTime;
    rampReset(this);
}

void rampReset(RampGenerator *this)
{
    this->upCounter = 0;
    this->upTime = 0;
    this->finalValue = 0.0;
    this->initialValue = 0.0;
    this->previousOutput = 0.0;
    this->deltaTime = 0.0;
}

void rampSet(RampGenerator *this, const float upTime, const float finalValue)
{
    this->upTime = upTime;
    this->finalValue = finalValue;
    this->initialValue = this->previousOutput;
    this->upCounter = roundf(this->upTime / this->samplingTime);
    this->deltaTime = (this->samplingTime / this->upTime) * (this->finalValue - this->initialValue);
}

float ramp(RampGenerator *this)
{
    if (this->upCounter)
    {
        this->previousOutput = this->finalValue - this->upCounter * this->deltaTime;
        this->upCounter--;
    }
    else
    {
        this->previousOutput = this->finalValue;
    }
    return this->previousOutput;
}

//------------------------------------------------------------------------------
void integratorInit(Integrator *this, const float samplingTime)
{
    this->samplingTime = samplingTime;
    integratorReset(this);
}

void integratorReset(Integrator *this)
{
    this->previousInput = 0.0;
    this->previousOutput = 0.0;
}

// Integrates using Tustin discretization method
float integrate(Integrator *this, const float input)
{
    this->previousOutput += (this->samplingTime / 2.0) * (input + this->previousInput);
    this->previousInput = input;
    return this->previousOutput;
}

//------------------------------------------------------------------------------
void diffInit(Differentiator *this, const float samplingTime)
{
    this->samplingTime = samplingTime;
    diffReset(this);
}

void diffReset(Differentiator *this)
{
    this->previousInput = 0.0;
}

// Differentiates using Backward discretization method
float diff(Differentiator *this, const float input)
{
    float output = (input - this->previousInput) / this->samplingTime;
    this->previousInput = input;
    return output;
}

//------------------------------------------------------------------------------
void filterInit(FirstOrderFilter *this, const float zero, const float pole, const float samplingTime)
{
    this->samplingTime = samplingTime;
    filterReset(this);
    filterDiscreet(this, zero, pole);
#ifdef SAVE_CONTINUOS_PARAMETERS
    this->pole = pole;
    this->zero = zero;
#endif
}

void filterReset(FirstOrderFilter *this)
{
    this->previousInput = 0.0;
    this->previousOutput = 0.0;
}

// Determines the filter coefficients using the Tustin discretization method
void filterDiscreet(FirstOrderFilter *this, const float zero, const float pole)
{
    this->coef[0] = (2.0 - pole * this->samplingTime) / (2.0 + pole * this->samplingTime);
    this->coef[1] = ((2.0 * (pole / zero) + pole * this->samplingTime) / (2.0 + pole * this->samplingTime));
    this->coef[2] = ((-2.0 * (pole / zero) + pole * this->samplingTime) / (2.0 + pole * this->samplingTime));
}

float filter(FirstOrderFilter *this, const float input)
{
    this->previousOutput = this->coef[0] * this->previousOutput + this->coef[1] * input + this->coef[2] * this->previousInput;
    this->previousInput = input;
    return this->previousOutput;
}

//------------------------------------------------------------------------------
/* This is meant to be the minimalistic implemensamplingTimetion
 * of a first order filter, there is only one pole and
 * it uses the simpler forward discretization method
 */
void sFilterInit(SimplifiedFirstOrderFilter *this, float pole, const float samplingTime)
{
    this->samplingTime = samplingTime;
    sFilterReset(this);
    sFilterDiscreet(this, pole);
#ifdef SAVE_CONTINUOS_PARAMETERS
    this->pole = pole;
#endif
}

void sFilterReset(SimplifiedFirstOrderFilter *this)
{
    this->previousOutput = 0.0;
}

// Determines the filter coefficients using the Forward discretization method
void sFilterDiscreet(SimplifiedFirstOrderFilter *this, const float pole)
{
    this->coef[1] = pole * this->samplingTime;
    this->coef[0] = 1.0 - this->coef[1];
}

float sFilter(SimplifiedFirstOrderFilter *this, const float input)
{
    this->previousOutput = this->coef[0] * this->previousOutput + this->coef[1] * input;
    return this->previousOutput;
}

//------------------------------------------------------------------------------
// Initializes PI controller
void piInit(PI *this, const float samplingTime)
{
    this->samplingTime = samplingTime;
    piReset(this);
}

void piReset(PI *this)
{
    this->controlSignal = 0.0;
    this->previousError = 0.0;
}

// Determines the discrete gains using the Tustin discretization method
void piDiscreet(PI *this, const float Kp, const float Ti)
{
    this->Kc = Kp * (2.0 + this->samplingTime / Ti) / (2.0);
    this->zc = (2.0 - this->samplingTime / Ti) / (2.0 + this->samplingTime / Ti);
}

// Design controller using pole zero cancellation
void piPoleZeroCancelationProject(PI *this, const float wmf, const float Kma, const float wma)
{
    float Ti = 1.0 / wma;
    float Kp = (wmf * Ti) / Kma;
    piDiscreet(this, Kp, Ti);
#ifdef SAVE_CONTINUOS_PARAMETERS
    this->wmf = wmf;
    this->Ti = Ti;
    this->Kp = Kp;
#endif
}

// Design controller for desired closed loop characteristics
void piClosedLoopResponseProject(PI *this, const float Mov, const float ts2, const float Kma, const float wma)
{
    // Desired closed loop characteristics
    float xi = fabsf(logf(Mov / 100.0) / sqrtf(square(logf(Mov / 100.0)) + square(M_PI)));
    float wn = 4.0 / (xi * ts2);
    // Calculate the analog gains
    float Kp = (2.0 * xi * wn - wma) / (Kma * wma);
    float Ti = (2.0 * xi * wn - wma) / (square(wn));
    piDiscreet(this, Kp, Ti);
#ifdef SAVE_CONTINUOS_PARAMETERS
    this->xi = xi;
    this->wn = wn;
    this->Ti = Ti;
    this->Kp = Kp;
#endif
}

// Calculate control output
float piControl(PI *this, const float setPoint, const float feedBack)
{
    float error = setPoint - feedBack;
    this->controlSignal = this->controlSignal + this->Kc * (error - this->zc * this->previousError);
    this->previousError = error;
#ifdef USE_SATURATORS
    this->controlSignal = appplySaturator(&this->sat, this->controlSignal);
#endif
    return this->controlSignal;
}

//------------------------------------------------------------------------------
// Initializes PI_D controller
void pidInit(PI_D *this, const float samplingTime)
{
    this->samplingTime = samplingTime;
    pidReset(this);
}

void pidReset(PI_D *this)
{
    for (uint16_t i = 0; i < 3; i++)
    {
        this->feedBack[i] = 0.0;
        this->error[i] = 0.0;
        this->controlSignal[i] = 0.0;
    }
#ifdef USE_PRE_FILTER
    this->spFiltered = 0.0;
    filterReset(&this->preFilter);
#endif
}

// Determines the discrete gains using the Tustin discretization method
void pidDiscreet(PI_D *this, const float Kp, const float Ti, const float Td, const float N)
{
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
void pidClosedLoopResponseProject(PI_D *this, const float Mov, const float ts2, const float Kma, const float wma)
{
    float xi, wn, real, imag, minZ, maxZ, zero, Ti, Kp, Td;
    // Derivative filter coefficient
    const float N = 10.0;
    // Desired closed loop characteristics
    xi = fabsf(logf(Mov / 100.0) / sqrtf(square(logf(Mov / 100.0)) + square(M_PI)));
    wn = 4.0 / (xi * ts2);
    real = fabsf(xi * wn);
    imag = fabsf(wn * sqrtf(1 - square(xi)));
    // Determines the closed-loop zero location
    minZ = wn / (2.0 * xi);
    maxZ = square(wn) / (2.0 * xi * wn - wma);
    zero = sqrtf(minZ * maxZ);
    // Calculate the analog gains
    Ti = 1.0 / zero;
    Kp = wn / (Kma * (2.0 * xi * zero - wn));
    Td = zero / square(wn) - 1.0 / (Kp * Kma * wma);
    pidDiscreet(this, Kp, Ti, Td, N);
#ifdef USE_PRE_FILTER
    // Project the pre filter
    filterInit(&this->preFilter, 10.0 * real, zero, this->samplingTime);
#endif
#ifdef SAVE_CONTINUOS_PARAMETERS
    this->xi = xi;
    this->wn = wn;
    this->real = real;
    this->imag = imag;
    this->minZ = minZ;
    this->maxZ = maxZ;
    this->zero = zero;
    this->Ti = Ti;
    this->Kp = Kp;
    this->Td = Td;
    this->N = N;
#endif
}

// Calculate the control output
float pidControl(PI_D *this, const float setPoint, const float feedBack)
{
    this->feedBack[2] = feedBack;
#ifdef USE_PRE_FILTER
    this->spFiltered = filter(&this->preFilter, setPoint);
    this->error[2] = this->spFiltered - feedBack;
#else
    this->error[2] = setPoint - feedBack;
#endif
    // Control signal
    this->controlSignal[2] = this->coef[0] * this->controlSignal[1] + this->coef[1] * this->controlSignal[0];
    for (uint16_t i = 0; i < 3; i++)
    {
        this->controlSignal[2] += this->coef[7 - i] * this->feedBack[i];
        this->controlSignal[2] += this->coef[4 - i] * this->error[i];
    }
#ifdef USE_SATURATORS
    this->controlSignal[2] = appplySaturator(&this->sat, this->controlSignal[2]);
#endif
    // Apply delay to internal buffers
    for (uint16_t i = 0; i < 2; i++)
    {
        this->feedBack[i] = this->feedBack[i + 1];
        this->error[i] = this->error[i + 1];
        this->controlSignal[i] = this->controlSignal[i + 1];
    }
    return this->controlSignal[2];
}

//------------------------------------------------------------------------------
void firstOrderObserverInit(FirstOrderObserver *this, const float Kma, const float wma, const float samplingTime)
{
    this->samplingTime = samplingTime;
    firstOrderObserverProject(this, Kma, wma);
    firstOrderObserverReset(this);
}

void firstOrderObserverReset(FirstOrderObserver *this)
{
    this->previousInput = 0.0;
    this->previousOutput = 0.0;
}

// FirstOrderObserver design (ZOH approach)
void firstOrderObserverProject(FirstOrderObserver *this, const float Kma, const float wma)
{
    this->zd = expf(-this->samplingTime * wma);
    this->Kd = Kma * (1 - this->zd);
#ifdef SAVE_CONTINUOS_PARAMETERS
    this->Kma = Kma;
    this->wma = wma;
#endif
}

float firstOrderObserve(FirstOrderObserver *this, const float input)
{
    this->previousOutput = this->zd * this->previousOutput + this->Kd * (this->previousInput);
    this->previousInput = input;
    return this->previousOutput;
}

//------------------------------------------------------------------------------
// END
//------------------------------------------------------------------------------
