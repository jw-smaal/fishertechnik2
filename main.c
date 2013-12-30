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
    
    /*
     * INIT ADC
     */
    ADCSRA |= (1<<ADEN);
    // ADMUX &= ˜(1 <<ADLAR);
    ADMUX = 0x00;
    
    
    
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
   // uint16_t adc_value;
    uint8_t detected_color;
    
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
        
        // motorTurnSteps(RIGHT, 5);
        motorTurn(RIGHT);
        _delay_ms(460); // Had issues with counting the impulses; this is more stable.
        motorOff();
        
        // We have arrived at the color sensor
        hefboomDown();
        _delay_ms(2000);
        
        detected_color = readColorSensor();
        
        hefboomUp();
        _delay_ms(400);
      
        switch(detected_color){
            case RED:
                motorTurnSteps(RIGHT,8);
                hefboomDown();
                _delay_ms(400);
                break;
            case WHITE:
                _delay_ms(400);
                motorTurnSteps(RIGHT,6);
                hefboomDown();
                _delay_ms(400);
                break;
            case BLUE:
                motorTurnSteps(RIGHT,4);
                hefboomDown();
                _delay_ms(400);
                break;
            default:
                hefboomDown();
                _delay_ms(400);
                break;
        }
        
        vacuumOff();
        _delay_ms(80);
        
        hefboomUp();
        _delay_ms(300);
        compressorOff();
        
        allOff();
    }
}
/* EOF */
