//------------------------------------------------------------------------------
#ifndef __CONTROLLERS_H
#define __CONTROLLERS_H

//------------------------------------------------------------------------------
// LIBRARIES
//------------------------------------------------------------------------------

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
#ifdef SAVE_CONTINUOS_PARAMETERS
    float zero;
    float pole;
#endif
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
#ifdef SAVE_CONTINUOS_PARAMETERS
    float pole;
#endif
} SimplifiedFirstOrderFilter;

typedef struct
{
    float controlSignal;
    float previousError;
#ifdef USE_SATURATORS
    Saturator sat;
#endif
    // Constants
    float samplingTime;
    float Kc;
    float zc;
#ifdef SAVE_CONTINUOS_PARAMETERS
    float wmf;
    float xi;
    float wn;
    float Kp;
    float Ti;
#endif
} PI;

typedef struct
{
    float feedBack[3];
    float error[3];
    float controlSignal[3];
#ifdef USE_PRE_FILTER
    float spFiltered;
    FirstOrderFilter preFilter;
#endif
#ifdef USE_SATURATORS
    Saturator sat;
#endif
    // Constants
    float samplingTime;
    float coef[8];
#ifdef SAVE_CONTINUOS_PARAMETERS
    float xi;
    float wn;
    float real;
    float imag;
    float minZ;
    float maxZ;
    float zero;
    float Kp;
    float Ti;
    float Td;
    float N;
#endif
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
#ifdef SAVE_CONTINUOS_PARAMETERS
    float Kma;
    float wma;
#endif
} FirstOrderObserver;

//------------------------------------------------------------------------------
// FUNCTION PROTOTYPES
//------------------------------------------------------------------------------

void saturatorSet(Saturator *this, const float max, const float min);
float appplySaturator(Saturator *this, const float in);

void rampInit(RampGenerator *this, const float samplingTime);
void rampReset(RampGenerator *this);
void rampSet(RampGenerator *this, const float upTime, const float finalValue);
float ramp(RampGenerator *this);

void integratorInit(Integrator *this, const float samplingTime);
void integratorReset(Integrator *this);
float integrate(Integrator *this, const float input);

void diffInit(Differentiator *this, const float samplingTime);
void diffReset(Differentiator *this);
float diff(Differentiator *this, const float input);

void filterInit(FirstOrderFilter *this, const float zero, const float pole, const float samplingTime);
void filterReset(FirstOrderFilter *this);
void filterDiscreet(FirstOrderFilter *this, const float zero, const float pole);
float filter(FirstOrderFilter *this, const float input);

/* This is meant to be the minimalistic implemensamplingTimetion
 * of a first order filter, there is only one pole and
 * it uses the simpler forward discretization method
 */
void sFilterInit(SimplifiedFirstOrderFilter *this, const float pole, const float samplingTime);
void sFilterReset(SimplifiedFirstOrderFilter *this);
void sFilterDiscreet(SimplifiedFirstOrderFilter *this, const float pole);
float sFilter(SimplifiedFirstOrderFilter *this, const float input);

void piInit(PI *this, const float samplingTime);
void piReset(PI *this);
void piDiscreet(PI *this, const float Kp, const float Ti);
void piPoleZeroCancelationProject(PI *this, const float wmf, const float Kma, const float wma);
void piClosedLoopResponseProject(PI *this, const float Mov, const float ts2, const float Kma, const float wma);
float piControl(PI *this, const float setPoint, const float feedBack);

void pidInit(PI_D *this, const float samplingTime);
void pidReset(PI_D *this);
void pidDiscreet(PI_D *this, const float Kp, const float Ti, const float Td, const float N);
void pidClosedLoopResponseProject(PI_D *this, const float Mov, const float ts2, const float Kma, const float wma);
float pidControl(PI_D *this, const float setPoint, const float feedBack);

void firstOrderObserverInit(FirstOrderObserver *this, const float Kma, const float wma, const float samplingTime);
void firstOrderObserverReset(FirstOrderObserver *this);
void firstOrderObserverProject(FirstOrderObserver *this, const float Kma, const float wma);
float firstOrderObserve(FirstOrderObserver *this, const float input);

#endif // __CONTROLLERS_H

//------------------------------------------------------------------------------
// END
//------------------------------------------------------------------------------
