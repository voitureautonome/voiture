#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "pca9685.h"
#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 60
#define M_PI 3.14159265358979323846

/**
 * Calculate the number of ticks the signal should be high for the required amount of time
 */
int calcTicks(float impulseMs, int hertz)
{
    float cycleMs = 1000.0f / hertz;
    return (int)(MAX_PWM * impulseMs / cycleMs + 0.5f);
}

/**
 * input is [0..1]
 * output is [min..max]
 */
float map(float input, float min, float max)
{
    return (input * max) + (1 - input) * min;
}

float radToDeg(float rad);
int main(int argc, char *argv[])
{
    wiringPiSetup();

    int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
    if (fd < 0)
    {
        printf("Error in setup\n");
        return fd;
    }
    float r;
    if(argc<2)
        r=0.5f;
    else{
        r=(atof(argv[1])+45.f)/100;
    }
    float millis = map(r, 1, 2);
    int tick = calcTicks(millis, HERTZ);
    pwmWrite(PIN_BASE + 0, tick);
    delay(2000);
    return 0;
}

float radToDeg(float rad)
{
    return rad * (180 / M_PI);
}
