#ifndef _FISHERTECNIKMODEL_H
#define _FISHERTECHNIKMODEL_H
/**
 *******************************************************************
 * $Id:$
 * FISHERTECHNIK model 2: PNEUMATIC
 *
 * Sorts woodblocks based on colour. Uses TIMER, ADC and digital 
 * I/O driving solenoid pneumatic valves, motor.  Sensors: switches
 * and phototransistors.
 *
 * Jan-Willem Smaal <jws@gispen.org> 22 Nov 2014
 *******************************************************************
 */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
// For the <util/delay.h> function in this case the internal 1MHz clock is used.
#define F_CPU 1000000UL
#include <util/delay.h>


/* OUTPUTS */
#define OUT_COMPRESSOR PC3
#define OUT_VACUUM PC4
#define OUT_HEFBOOM PC5
#define OUT_MOTOR1A PD6
#define OUT_MOTOR2A PD7
#define OUT_MOTOR_EN PD5
#define OUT_LED PB0

/* INPUTS */
#define IN_COLOUR PC0 
#define IN_LIGHT PD0
#define IN_STEPS PD1 
#define IN_MOTOR_END PD2



/* Program Constants */
#define LEFT 0 
#define RIGHT 1

// Colours
#define INVALID 0
#define BLUE 1
#define WHITE 2
#define RED 3

/**
 * Prototypes:
 */
void flashntimes(int n);
void allOff(void);
void motorTurn(int direction);
void motorTurnSteps(int direction, int steps);
void motorOff(void);
void compressorOn(void);
void compressorOff(void);
void vacuumOn(void);
void vacuumOff(void);
void hefboomUp(void);
void hefboomDown(void);
void ledOn(void);
void ledOff(void);
void motorCountSteps(int steps);
uint8_t readColorSensor(void);

/* EOF */
#endif