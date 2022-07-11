//------------------------------------------------------------------------------
#ifndef __CONTROLLERS_H
#define __CONTROLLERS_H

//------------------------------------------------------------------------------
// LIBRARIES
//------------------------------------------------------------------------------

#include <stdbool.h>
#include <stdint.h>

//------------------------------------------------------------------------------
// CUSTOM TYPES
//------------------------------------------------------------------------------

typedef struct
{
    float max;
    float min;
} Saturator;

typedef struct
{
    float upTime;
    uint32_t upCounter;
    float finalValue;
    float initialValue;
    float previousOutput;
    float deltaTime;
    // Constants
    float samplingTime;
} RampGenerator;

// Integrator (Tustin's method)
typedef struct
{
    float previousInput;
    float previousOutput;
    // Constants
    float samplingTime;
} Integrator;

// Differentiator (Backward method)
typedef struct
{
    float previousInput;
    // Constants
    float samplingTime;
} Differentiator;

typedef struct
{
    float previousInput;
    float previousOutput;
    // Constants
    float samplingTime;
    float coef[3];
} FirstOrderFilter;

/* This is meant to be the minimalistic implemensamplingTimetion
 * of a first order filter, there is only one pole and
 * it uses the simpler forward discretization method
 */
typedef struct
{
    float previousOutput;
    // Constants
    float samplingTime;
    float coef[2];
} SimplifiedFirstOrderFilter;

typedef struct
{
    float controlSignal;
    float previousError;
    bool useSaturator;
    Saturator sat;
    // Constants
    float samplingTime;
    float Kc;
    float zc;
} PI;

typedef struct
{
    float feedBack[3];
    float error[3];
    float controlSignal[3];
    bool useSaturator;
    Saturator sat;
    bool usePreFilter;
    float spFiltered;
    FirstOrderFilter preFilter;
    // Constants
    float samplingTime;
    float coef[8];
} PI_D;

// 1st order observer
typedef struct
{
    float previousInput;
    float previousOutput;
    // Constants
    float samplingTime;
    float Kd;
    float zd;
} FirstOrderObserver;

//------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

void saturatorSet(Saturator *this, const float max, const float min);
float applySaturator(Saturator *this, const float in);

void rampInit(RampGenerator *this, const float samplingTime);
void rampReset(RampGenerator *this);
void rampSet(RampGenerator *this, const float upTime, const float finalValue);
float calcRamp(RampGenerator *this);

void integratorInit(Integrator *this, const float samplingTime);
void integratorReset(Integrator *this);
float integrate(Integrator *this, const float input);

void diffInit(Differentiator *this, const float samplingTime);
void diffReset(Differentiator *this);
float differentiate(Differentiator *this, const float input);

void filterInit(FirstOrderFilter *this, const float zero, const float pole, const float samplingTime);
void filterReset(FirstOrderFilter *this);
void filterDiscreet(FirstOrderFilter *this, const float zero, const float pole);
float filterProcess(FirstOrderFilter *this, const float input);

/* This is meant to be the minimalistic implementation
 * of a first order filter. There is only one pole and
 * it uses the simpler forward discretization method
 */
void sFilterInit(SimplifiedFirstOrderFilter *this, const float pole, const float samplingTime);
void sFilterReset(SimplifiedFirstOrderFilter *this);
void sFilterDiscreet(SimplifiedFirstOrderFilter *this, const float pole);
float sFilterProcess(SimplifiedFirstOrderFilter *this, const float input);

void piInit(PI *this, const float samplingTime, const bool useSaturator);
void piReset(PI *this);
void piDiscreet(PI *this, const float Kp, const float Ti);
void piPoleZeroCancelationProject(PI *this, const float wcl, const float Kol, const float wol);
void piClosedLoopResponseProject(PI *this, const float Mov, const float ts2, const float Kol, const float wol);
float piControl(PI *this, const float setPoint, const float feedBack);

void pidInit(PI_D *this, const float samplingTime, const bool useSaturator, const bool usePreFilter);
void pidReset(PI_D *this);
void pidDiscreet(PI_D *this, const float Kp, const float Ti, const float Td, const float N);
void pidClosedLoopResponseProject(PI_D *this, const float Mov, const float ts2, const float Kol, const float wol);
float pidControl(PI_D *this, const float setPoint, const float feedBack);

void observerInit(FirstOrderObserver *this, const float Kol, const float wol, const float samplingTime);
void observerReset(FirstOrderObserver *this);
void observerProject(FirstOrderObserver *this, const float Kol, const float wol);
float observerProcess(FirstOrderObserver *this, const float input);

#endif  // __CONTROLLERS_H

//------------------------------------------------------------------------------
// END
//------------------------------------------------------------------------------
