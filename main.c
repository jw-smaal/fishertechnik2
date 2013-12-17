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
#include <avr/signature.h>
#include <avr/io.h>
#include <avr/interrupt.h>
// For the <util/delay.h> function in this case the internal 1MHz clock is used.
#define F_CPU 1000000UL
#include <util/delay.h>

/* Implementation of fishertechnik model */
#include "fishertechnikmodel.h"


void main(void) __attribute__((noreturn));

/**
 *******************************************************************
 * init_mcu(void) called only once after reset.
 *******************************************************************
 */
void init_mcu(void)
{
    int i;
    /*
     * INIT data direction register:
     */
    DDRB =  0b00000001; // LED
    DDRC =  0b00111000; // Vacuum
    DDRD =  0b11100000; // Motor
    
    PORTC = 0b00000000; // Do not enable pullup on the ADC port.
    PORTD = 0b00000111; // Enable 20k pullup resistor on pinD 0,1 and 2
    
    /*
     * INIT timer registers:
     * 1MHz clock / 64 = 15,625 KHz
     * 16bit TCNT1 counter overflows every (15,625 KHz/65535) = 0.23 times per second.
     */
    TCCR1B |= ( (1<<CS10) | (1<<CS11) ); // Use prescale divided by 64.
    
    // This ledsequence is to indicate power-on-reset.
    allOff();
    for(i = 0; i < 5; i++){
        ledOn();
        _delay_ms(100);
        ledOff();
        _delay_ms(100);
    }
}


/** 
 *******************************************************************
 * main run loop; never exits!
 *******************************************************************
 */
void main(void)
{
    uint8_t light_sensor;
    uint8_t motor_end;
    
    init_mcu();
    
    // main run loop never exit!
    while(1) {
        
        // Wait for IN_LIGHT to be interupted (1 == interupted; 0 = open)
        do {
            light_sensor  = ( PIND & ( 1 << IN_LIGHT ) ) ? 1 : 0;
        }
        while (!light_sensor);
    
        compressorOn();
        
        // Check for MOTOR_END before starting the motor
        motor_end  = ( PIND & ( 1 << IN_MOTOR_END ) ) ? 0 : 1;
        if(!motor_end){
            do {
                // Keep turning the motor untill we reach the end
                motorTurn(LEFT);
                _delay_ms(10);
                motor_end  = ( PIND & ( 1 << IN_MOTOR_END ) ) ? 0 : 1;
            }
            while (!motor_end);
        }
        motorOff();
        
        // Now we can pickup the woodblock at this position
        hefboomDown();
        _delay_ms(200);
            
        vacuumOn();
        _delay_ms(400);
            
        hefboomUp();
        _delay_ms(300);
        
        motorTurnSteps(RIGHT, 4);
        // We have arrived at the sensor
        hefboomDown();
        _delay_ms(2000);
        /* 
         * TODO: insert colour sensor ADC conversion.
         * then hefboomUp(); then turn the motor to the right (n) steps
         * based on the colour.
         * finally hefboomDown()....
         */
        
        
        hefboomUp();
        _delay_ms(400);
      
        // TODO: Should be counting the impulses based on COLOUR selection done
        // at the ADC
        motorTurnSteps(RIGHT,5);
        
        // Lay down the block
        hefboomDown();
        _delay_ms(400);
        
#if 1
        hefboomUp();
        _delay_ms(400);
        motorTurnSteps(RIGHT,2);
        hefboomDown();
        _delay_ms(400);

        hefboomUp();
        _delay_ms(400);
        motorTurnSteps(RIGHT,2);
        hefboomDown();
        _delay_ms(400);
#endif
        
        vacuumOff();
        _delay_ms(80);
        
        hefboomUp();
        _delay_ms(300);
        compressorOff();
        
        allOff();
    }
}
/* EOF */
