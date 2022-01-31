# digital-control

## Overview

Thisis a simple C library containing very useful digital control and signal processing functionalities destinated for DSP's and microcontrollers. It posseses the following characteristics:

- Compatible with C99;
- It uses `float` variables (32 bits floating point precision), so it is recomended that it is used with a processor that has a [FPU](https://en.wikipedia.org/wiki/Floating-point_unit);
- This library simulates object oriented programming in C, in order to to simplify its use;
- It is recomended that you don't modify the variables inside the structs directly. Insted, use the provided functions to set the parameters, project the filters and controllers;
- The design parameters received by the functions exported by this library are all in the continuous domain. The discretization of these coefficients is performed by the functions themselves, using the appropriate discretization method for each case;
- This library was used in the development of the following projects:
    - [Melhoria da robustez e eficiência em acionamentos de motores de indução trifásicos combinando as técnicas IFOC, LMC e MRAC](https://repositorio.utfpr.edu.br/jspui/bitstream/1/24529/1/melhoriarobustezeficienciamotores.pdf) (in brazilian portuguese);
    - [Adaptive Loss Model Control for robustness and efficiency improvement of induction motor drive](https://ieeexplore.ieee.org/abstract/document/9612111);
- This library includes the following implementations:
    - [Saturator](#Saturator);
    - [Ramp](#Ramp);
    - [Integrator](#Integrator);
    - [Differentiator](#Differentiator);
    - [First Order Filter](#First-Order-Filter);
    - [Simplified First Order Filter](#Simplified-First-Order-Filter);
    - [Proportional Integral (PI) Control](#Proportional-Integral-PI-Control);
    - [Proportional Integral Derivative (PID) Control](#Proportional-Integral-Derivative-PID-Control);
    - [First Order Observer](#First-Order-Observer);

## Usage

If you wish to use this library on your project, just download the header file `digitalControl.h` and source file `digitalControl.c`, and add them to your project folder including the header file to your c code, just like this:

```c
#include "digitalControl.h"
```

This library also accept tree optional configurations, which are informed by preprocessor macros, just as in this example:

```c
// Saves in the structs the continuous parameters, just like poles,
// continuous controller gains, etc.
#define SAVE_CONTINUOS_PARAMETERS
// Uses saturators on the PI and PI_D controllers
#define USE_SATURATORS
// Uses pre-filter on the reference (setpoint) for the PI_D controller
#define USE_PRE_FILTER

#include "digitalControl.h"
```

Following, more detailed description of each functionality of this library are provided.

### Saturator

The saturator object is designed to apply maximum and minimum limits to a signal. It may be used according to the following example:

```c
float min = 0.0f;
float max = 100.0f;
float input = 10.0f;
// Creates a saturator object
Saturator sat;
// Configures the saturator object with a maximum and minimum value
saturatorSet(&sat, max, min);
// Applies the saturator limits to the input value
float out = applySaturator(&sat, input);
```

### Ramp

A ramp is a structure used to avoid sudden variations in a signal, for example the reference of a controller. To do this, when a change in signal amplitude is requested, the ramp makes this signal grow linearly over a specified time interval. The following example illustrates the use of this structure:

```c
const float samplingTime = 0.01f;
float upTime = 1.0f;
float finalValue = 100.0f;
// Creates a ramp object and initializes it
RampGenerator ramp;
void rampInit(&ramp, samplingTime);
// Configures the ramp object to create a ramp which starts at 0.0f,
// and ends at finalValue, in the period provided by upTime
void rampSet(&ramp, upTime, finalValue);
// Calculate the new ramp value. This function must be called
// periodically, with the period provided by samplingTime
float out = calcRamp(&ramp);
```

If you wish to change the output signal once again, you can use the same `ramp` object with a new configuration, like so:

```c
float newUpTime = 1.0f;
float newFinalValue = 50.0f;
// The new ramp will start at is previous value and end at newFinalValue,
// and this change will take newUpTime units of time
void rampSet(&ramp, newUpTime, newFinalValue);
// Calculate the new ramp value
float out = calcRamp(&ramp);
```

### Integrator

The [Integrator](https://en.wikipedia.org/wiki/Integrator) object is designed to provide as its output signal the time integral of its input signal. This algorithm is implemented using the [Tustin's discretization method](https://en.wikipedia.org/wiki/Bilinear_transform) (also known as bilinear transform). Here is an simple example:

```c
const float samplingTime = 0.01f;
float input  = 0.0f;
// Creates an integrator object and initializes it
Integrator int;
integratorInit(&int, samplingTime);
// Calculate the integrator output. This function must be called
// periodically, with the period provided by samplingTime
float out = integrate(&int, input);
```

### Differentiator

The [Differentiator](https://en.wikipedia.org/wiki/Differentiator) object is designed to provide as its output signal the time derivative of its input signal. This algorithm is implemented using the [Backward discretization method](https://en.wikipedia.org/wiki/Backward_Euler_method). Take a look at the following example:

```c
const float samplingTime = 0.01f;
float input  = 0.0f;
// Creates an differentiator object and initializes it
Differentiator diff;
diffInit(&diff, samplingTime);
// Calculate the differentiator output. This function must be called
// periodically, with the period provided by samplingTime
float out = differentiate(&diff, input);
```

### First Order Filter

This object consists of a first order filter having a pole and a zero, both specified at its initialization (this filter has unitary gain). This algorithm is implemented using the [Tustin's discretization method](https://en.wikipedia.org/wiki/Bilinear_transform) (also known as bilinear transform). Check out this example:

```c
const float samplingTime = 0.01f;
float zero  = 1.0f;
float pole  = 10.0f;
float input  = 0.0f;
// Creates an filter object and initializes it
FirstOrderFilter filter;
filterInit(&filter, zero, pole, samplingTime);
// Calculate the filter output. This function must be called
// periodically, with the period provided by samplingTime
float out = filterProcess(&filter, input);
```

If at some point you wish to reset the internal state of the created filter, you can do this by calling the following function:

```c
filterReset(&filter);
```

You may also change the filter coefficients at latter time, by calling this function:

```c
filterDiscreet(&filter, zero, pole);
```

The Laplace transfer function implemented by this filter is:

<img src="https://render.githubusercontent.com/render/math?math=\color{blue} F(s) = \displaystyle\frac{1 %2B s/zero}{1 %2B s/pole}">

### Simplified First Order Filter

The `SimplifiedFirstOrderFilter` object was designed to be a simplified version of the previously presented filter (it consumes slightly less memory, and requires less processing). It is also a first order filter, but its zero cannot be specified, always having a null value. This implementation uses the simpler [Forward discretization method](https://en.wikipedia.org/wiki/Euler_method) (also known as Euler method). here is an example of usage:

```c
const float samplingTime = 0.01f;
float pole  = 10.0f;
float input  = 0.0f;
// Creates an filter object and initializes it
SimplifiedFirstOrderFilter filter;
sFilterInit(&filter, pole, samplingTime);
// Calculate the filter output. This function must be called
// periodically, with the period provided by samplingTime
float out = sFilterProcess(&filter, input);
```

You can reset the internal state of the `SimplifiedFirstOrderFilter` object by calling the following function:

```c
sFilterReset(&filter);
```

You may also change the filter coefficients at latter time, by calling this function:

```c
sFilterDiscreet(&filter, pole);
```

The Laplace transfer function implemented by this filter is:

<img src="https://render.githubusercontent.com/render/math?math=\color{blue} F(s) = \displaystyle\frac{1}{1 %2B s/pole}">

### Proportional Integral (PI) Control

The `PI` object is a simple implementation of the PI controller in the academic form. This algorithm is implemented using the [Tustin's discretization method](https://en.wikipedia.org/wiki/Bilinear_transform) (also known as bilinear transform). Check out this example:

```c
const float samplingTime = 0.01f;
float setPoint = 10.0f;
float feedBack = 0.0f;
float Kp = 10.0f; // Proportional gain
float Ti = 0.1f; // Integral time
// Creates an PI object and initializes it
PI pi;
piInit(&pi, samplingTime);
// Inform the continuous gains to the PI struct
piDiscreet(&pi, Kp, Ti);
// Calculate the PI output. This function must be called
// periodically, with the period provided by samplingTime
float out = piControl(&pi, setPoint, feedBack);
```

You can reset the internal state of this controller by calling the following function:

```c
piReset(&pi);
```

This library also provides tho functions to automatically calculate its gains. The first of this functions receives the desired closed loop bandwidth (`wcl`), the open loop bandwidth (`wol`) and the open loop gain (`Kol`) of your system and project the controller using pole-zero cancelation method. here is a usage example (if you use this function, you don't call the function `piDiscreet`):

```c
piPoleZeroCancelationProject(&pi, wcl, Kol, wol);
```

Another project possibility is to design the controller for a desired second order step response. In this case you use the function `piClosedLoopResponseProject` and provide as arguments the accepted overshoot percentage (`Mov`), 2% settling time (`ts2`), alongside to the the open loop bandwidth (`wol`) and the open loop gain (`Kol`):

```c
piClosedLoopResponseProject(&pi, Mov, ts2, Kol, wol);
```

If you wish to use saturators in this controler, you must define `USE_SATURATORS` before including the library header file in your project, and must also initialize the `PI`  internal saturator, like so:

```c
#define USE_SATURATORS
#include "digitalControl.h"

// Creates an PI object and initializes it
PI pi;
piInit(&pi, samplingTime);
// Initializes the PI internal saturator
saturatorSet(&pi.sat, max, min);
// Projects the PI controller by the desired second order response
piClosedLoopResponseProject(&pi, Mov, ts2, Kol, wol);
// Calculate the PI output. This function must be called
// periodically, with the period provided by samplingTime
float out = piControl(&pi, setPoint, feedBack);
```

The Laplace transfer function implemented by this PI controller is:

<img src="https://render.githubusercontent.com/render/math?math=\color{blue} F(s) = K_p \left(\displaystyle\frac{T_i s %2B 1}{T_i s}\right)">

### Proportional Integral Derivative (PID) Control

The `PI_D` object defines an implementation of the PI-D controller in the academic form (it posseses filter in the derivative action and the derivative action acts on the feedback signal and not on the error). This algorithm is implemented using the [Tustin's discretization method](https://en.wikipedia.org/wiki/Bilinear_transform) (also known as bilinear transform). Check out this example:

```c
const float samplingTime = 0.01f;
float setPoint = 10.0f;
float feedBack = 0.0f;
float Kp = 10.0f; // Proportional gain
float Ti = 0.1f; // Integral time
float Td = 0.1f; // Derivative time
float N = 10.0f; // Derivative filter coefficient
// Creates an PI_D object and initializes it
PI_D pid;
pidInit(&pid, samplingTime);
// Inform the continuous gains to the PI_D struct
pidDiscreet(&pid, Kp, Ti, Td, N);
// Calculate the PI_D output. This function must be called
// periodically, with the period provided by samplingTime
float out = pidControl(&pid, setPoint, feedBack);
```

You can reset the internal state of this controller by calling the following function:

```c
pidReset(&pid);
```

Just as in the PI controller case, you can design this controller for a desired second order step response. In this case you use the function `pidClosedLoopResponseProject` and provide as arguments the accepted overshoot percentage (`Mov`), 2% settling time (`ts2`), the open loop bandwidth (`wol`) and the open loop gain (`Kol`). IN this case you don't call the `pidDiscreet` function. Here is an example:

```c
pidClosedLoopResponseProject(&pid, Mov, ts2, Kol, wol);
```

If you wish to use saturators in this controler, you must define `USE_SATURATORS` before including the library header file in your project, and must also initialize the `PI_D`  internal saturator, just like in the `PI` case, so no example will be provided.

If you define the macro `USE_PRE_FILTER` before including the library header file, the `PI_D` controller also will posses an internal pre-filter applied to the reference (setpoint). Here is an example of usage in this case:

```c
#define USE_PRE_FILTER
#include "digitalControl.h"

// Creates an PI_D object and initializes it
PI_D pid;
pidInit(&pid, samplingTime);
// Initializes the internal pre-filter
// This step can be skipped if you use the pidClosedLoopResponseProject
// function, because it initializes the filter internally
filterInit(&pid.preFilter, pole, zero, samplingTime);
// Inform the continuous gains to the PI_D struct
pidDiscreet(&pid, Kp, Ti, Td, N);
// Calculate the PI_D output. This function must be called
// periodically, with the period provided by samplingTime
float out = pidControl(&pid, setPoint, feedBack);
```

Since the output of this controller depends both on the error and the feedback, it cannot be described by a single Laplace transfer function. The output of this PI-D controller (denoted by `U(s)`) can be determined from the error (`E(s)`) and the feedback (`F(s)`), using the following Laplace expression:

<img src="https://render.githubusercontent.com/render/math?math=\color{blue} U(s) = K_p \left(\displaystyle\frac{T_i s %2B 1}{T_i s}\right) E(s) - K_p \left(\displaystyle\frac{T_d s}{\left(T_d / N \right) s %2B 1}\right) F(s)">

### First Order Observer

This object defines an estimator/observer of a first order system. If you know the open loop gain and the open loop bandwidth of a first order system, this struct may help you estimate its output signal from the input provided to this system. This algorithm was discretized using the [Zero-order hold approach](https://en.wikipedia.org/wiki/Zero-order_hold). Here is an example of usage:


```c
const float samplingTime = 0.01f;
float Kol = 10.0f; // Open loop gain
float wol = 100.0f; // Open loop bandwidth (rad/s)
// Creates an observer object and initializes it
FirstOrderObserver observer;
observerInit(&observer, Kol, wol, samplingTime);
// Calculate the observer output. This function must be called
// periodically, with the period provided by samplingTime
float out = observerProcess(&observer, input);
```

You can reset the internal state of the observer by calling the following function:

```c
observerReset(&observer);
```

You may also change the observer coefficients at latter time, by calling this function:

```c
observerProject(&observer, Kol, wol);
```

The Laplace transfer function estimated by this observer is:

<img src="https://render.githubusercontent.com/render/math?math=\color{blue} O(s) = \displaystyle\frac{K_{ol}}{s %2B w_{ol}}">

More information about the functioning of this library can be achieved by consulting the source code (its not that hard). Feel free to contact me with questions or comments.
