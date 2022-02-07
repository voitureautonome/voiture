#include <cstdio>
#include <iostream>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include "pca9685.h"
#define PIN_BASE 300
#define MAX_PWM 4096
#define HERTZ 60

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
int main(int argc, char *argv[])
{
    wiringPiSetup();
    pinMode(23, OUTPUT);
	pinMode(0, OUTPUT);
	pinMode(2, OUTPUT);
	digitalWrite(0, 1);
	digitalWrite(2, 1);
    int fd = pca9685Setup(PIN_BASE, 0x40, HERTZ);
    if (fd < 0)
    {
        printf("Error in setup\n");
        return fd;
    }
    // Reset all output
    pca9685PWMReset(fd);

    float millis = 1.5;
    int tick = calcTicks(millis, HERTZ);
    pwmWrite(PIN_BASE + 16, tick);
	//wiringPiI2CSetup(0x48);
	//wiringPiI2CWriteReg8(0x48, 0x40, 1);
	int i, j = 1;
	int pin;
	pwmWrite(PIN_BASE + 4, calcTicks(12, HERTZ));
	pwmWrite(PIN_BASE + 5, calcTicks(12, HERTZ));
    if(argc<2)
        delay(1000);
    else
        delay(atoi(argv[1]));
    pwmWrite(PIN_BASE + 4, calcTicks(0, HERTZ));
	pwmWrite(PIN_BASE + 5, calcTicks(0, HERTZ));
    return 0;
}
