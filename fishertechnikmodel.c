/**
 *******************************************************************
 * $Id:$
 * FISHERTECHNIK model 2: PNEUMATIC
 *
 * Sorts woodblocks based on colour. Uses TIMER, ADC and digital 
 * I/O driving solenoid pneumatic valves, motor.  Sensors: switches
 * and phototransistors.
 *
 * Jan-Willem Smaal <jws@gispen.org> 22 Nov 2013
 *******************************************************************
 */
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
// For the <util/delay.h> function in this case the internal 1MHz clock is used.
#define F_CPU 1000000UL
#include <util/delay.h>

#include "fishertechnikmodel.h"

/**
 *******************************************************************
 * VARIOUS FisherTechnik related functions for controlling the model
 * all return void because there is nothing to check 
 * just reading/writing to memory registers.
 *******************************************************************
 */
void flashntimes(int n){
    int i = 0;
    for (i = 0; i < n; i++){
        ledOn();
        _delay_ms(600);
        ledOff();
        _delay_ms(400);
    }
}

void allOff()
{
    motorOff();
    vacuumOff();
    ledOff();
    hefboomUp();
    compressorOff();
}


void motorTurnSteps(int direction, int steps){
    motorTurn(direction);
    motorCountSteps(steps);
    motorOff();
}

void motorTurn(int direction)
{
    if(direction == LEFT) {
        PORTD &= ~(1<<OUT_MOTOR2A); // Clear bit
        PORTD |= ( (1<<OUT_MOTOR_EN) | (1<<OUT_MOTOR1A) ); // Set bit
    }
    else {  // Right
        PORTD &= ~(1<<OUT_MOTOR1A);
        PORTD |= ( (1<<OUT_MOTOR_EN) | (1<<OUT_MOTOR2A) );
    }
}

void motorOff(void)
{
    PORTD &= ~(1<<OUT_MOTOR_EN); // clear enable bit
}

void compressorOn(void)
{
    PORTC |= (1<<OUT_COMPRESSOR);
}

void compressorOff(void)
{
    PORTC &= ~(1<<OUT_COMPRESSOR);
}

void vacuumOn(void)
{
    PORTC |= (1<<OUT_VACUUM);
}

void vacuumOff(void)
{
    PORTC &= ~(1<<OUT_VACUUM);
}

void hefboomUp(void)
{
    PORTC &= ~(1<<OUT_HEFBOOM);
}

void hefboomDown(void)
{
    PORTC |= (1<<OUT_HEFBOOM);
}

void ledOn(void)
{
    PORTB &= ~(1<<OUT_LED);
}

void ledOff(void)
{
    PORTB |= (1<<OUT_LED);
}

void motorCountSteps(int steps)
{
    uint8_t motor_steps;
    
    motor_steps = 0;
    do {
        // Logic 1 (no step), logic 0 (step)
        while((PIND & (1<<IN_STEPS)) ? 1: 0 ){
            // wait for falling edge
        }
        motor_steps++;
        if(motor_steps == steps) {
            return; // Don't wait for rising edge
        }
        _delay_ms(10); // De-bounce
        while((PIND & (1<<IN_STEPS)) ? 0: 1 ){
            // wait for rising edge
        }
        motor_steps++;
        if(motor_steps == steps){
            return; 
        }
        //_delay_ms(1); // De-bounce
    } while (motor_steps < steps);
}


uint8_t readColorSensor(void)
{
    uint16_t adc_value;
    uint8_t detected_color = INVALID;
    /**
     * Colour sensor ADC conversion.
     */
    ADCSRA |= (1<<ADSC); // start conversion
    while((ADCSRA & (1<<ADSC))==1<<ADSC) {
        // wait for conversion to complete.
    }
    adc_value = ADCL; // first read ADCL then ADCH; read datasheet!
    adc_value |= ADCH<<8 ; // 2.91V = open;
    // 2.93V = blue;
    // 2.56V = red;
    // 2.48V = white;
    if(adc_value <= (1024/5)*2.53){
        // WHITE
        detected_color = WHITE;
        flashntimes(2);
    }
    else if(adc_value >= (1024/5)*2.56  && adc_value < (1024/5)*2.80) {
        // RED
        detected_color = RED;
        flashntimes(1);
    }
    else if(adc_value > (1024/5)*2.90) {
        // BLUE
        detected_color = BLUE;
        flashntimes(3);
    }
    /* done with detection */
    return detected_color;
}

/* EOF */

