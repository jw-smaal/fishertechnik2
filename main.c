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


void main(void) __attribute__((noreturn));

/**
 * Prototypes:
 */
void flashntimes(int n);
void allOff(void);
void motorTurn(int direction);
void motorOff(void);
void compressorOn(void);
void compressorOff(void);
void vacuumOn(void);
void vacuumOff(void);
void hefboomUp(void);
void hefboomDown(void);
void ledOn(void);
void ledOff(void);


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
        
        motorTurn(RIGHT);
        // Should be counting the impulses.
        _delay_ms(500);  // 0->500ms:sensor  0->800ms:right sorter
        motorOff();
        
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
        
        motorTurn(RIGHT);
        // TODO: Should be counting the impulses based on COLOUR selection
        _delay_ms(300);
        motorOff();
        
        // Lay down the block
        hefboomDown();
        _delay_ms(400);
        
        vacuumOff();
        _delay_ms(80);
        
        hefboomUp();
        _delay_ms(300);
        compressorOff();
        
        allOff();
    }
}


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

/* EOF */
