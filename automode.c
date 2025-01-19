#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <builtins.h>

#define _XTAL_FREQ 4000000  // Internal Clock speed
#define LOW_SPEED 800
#define MID_SPEED 1000
#define HIGH_SPEED 1700
#define MOTOR_SPEED_OFFSET 350
#include "lib.h"

#pragma config OSC = INTIO67  // Oscillator Selection bits
#pragma config WDT = OFF      // Watchdog Timer Enable bit
#pragma config PWRT = OFF     // Power-up Enable bit
#pragma config BOREN = ON     // Brown-out Reset Enable bit
#pragma config PBADEN = OFF   // Watchdog Timer Enable bit
#pragma config LVP = OFF      // Low Voltage (single-supply) In-Circute Serial Pragramming Enable bit
#pragma config CPD = OFF      // Data EEPROM Memory Code Protection bit (Data EEPROM code protection off)
int mode = 0;
bool turrning = false;
int flash_state = 0;

unsigned int seed = 12345;

unsigned int generateRandom() {
    seed = (seed * 1103515245 + 12345) & 0x7FFFFFFF;
    return seed;
}
bool randomBinaryChoice() {
    return generateRandom() % 2;
}

void motor_stop() {
    setCCP1PwmDutyCycle(LOW_SPEED, 16);
    setCCP2PwmDutyCycle(LOW_SPEED, 16);
    digitalWrite(PIN_RD0, 1);
    digitalWrite(PIN_RD1, 1);
    digitalWrite(PIN_RD2, 1);
    digitalWrite(PIN_RD3, 1);
    return;
}
void move_forward() {
    digitalWrite(PIN_RD0, 0);
    digitalWrite(PIN_RD1, 1);
    digitalWrite(PIN_RD2, 0);
    digitalWrite(PIN_RD3, 1);
    setCCP1PwmDutyCycle(HIGH_SPEED + MOTOR_SPEED_OFFSET, 16);
    setCCP2PwmDutyCycle(HIGH_SPEED, 16);
    __delay_ms(100);
    setCCP1PwmDutyCycle(LOW_SPEED + MOTOR_SPEED_OFFSET, 16);
    setCCP2PwmDutyCycle(LOW_SPEED, 16);
    return;
}
void move_back() {
    digitalWrite(PIN_RD0, 1);
    digitalWrite(PIN_RD1, 0);
    digitalWrite(PIN_RD2, 1);
    digitalWrite(PIN_RD3, 0);
    setCCP1PwmDutyCycle(HIGH_SPEED + MOTOR_SPEED_OFFSET, 16);
    setCCP2PwmDutyCycle(HIGH_SPEED, 16);
    __delay_ms(100);
    setCCP1PwmDutyCycle(LOW_SPEED + MOTOR_SPEED_OFFSET, 16);
    setCCP2PwmDutyCycle(LOW_SPEED, 16);
    return;
}
void motor_turn_left() {
    digitalWrite(PIN_RD0, 1);
    digitalWrite(PIN_RD1, 0);
    digitalWrite(PIN_RD2, 0);
    digitalWrite(PIN_RD3, 1);
    setCCP1PwmDutyCycle(HIGH_SPEED + MOTOR_SPEED_OFFSET, 16);
    setCCP2PwmDutyCycle(HIGH_SPEED, 16);
    __delay_ms(100);
    setCCP1PwmDutyCycle(MID_SPEED + MOTOR_SPEED_OFFSET, 16);
    setCCP2PwmDutyCycle(MID_SPEED, 16);
    return;
}
void motor_turn_right() {
    digitalWrite(PIN_RD0, 0);
    digitalWrite(PIN_RD1, 1);
    digitalWrite(PIN_RD2, 1);
    digitalWrite(PIN_RD3, 0);
    setCCP1PwmDutyCycle(HIGH_SPEED + MOTOR_SPEED_OFFSET, 16);
    setCCP2PwmDutyCycle(HIGH_SPEED, 16);
    __delay_ms(100);
    setCCP1PwmDutyCycle(MID_SPEED + MOTOR_SPEED_OFFSET, 16);
    setCCP2PwmDutyCycle(MID_SPEED, 16);
    return;
}

void fan_toggle() {
    digitalWrite(PIN_RD5, !pinState(PIN_RD5));
    return;
}
void stop_all() {
    motor_stop();
    digitalWrite(PIN_RD5, 1);
    digitalWrite(PIN_RD6, 0);
    mode = 0;
}
void switch_state() {
    if (mode == 2) {
        stop_all();
    } else if (mode == 0) {
        mode = 1;
        digitalWrite(PIN_RD6, 1);
    } else if (mode == 1) {
        mode = 2;
        digitalWrite(PIN_RD6, 1);
        digitalWrite(PIN_RD5, 0);
    }
}
void onReadLine(char *line, byte len) {
    motor_stop();
    return;
}

void onReadChar(char c) {
    if (c == 'C')
        switch_state();

    if (mode == 0)
        return;
    if (c == 'R')
        motor_turn_right();
    else if (c == 'L')
        motor_turn_left();
    else if (c == 'F')
        move_forward();
    else if (c == 'B')
        move_back();
    else if (c == 'S')
        fan_toggle();
    else if (c == 'X')
        stop_all();
    return;
}

void __interrupt(high_priority) H_ISR() {
    if (interruptByRB0External()) {
        clearInterrupt_RB0External();
        switch_state();
    }
    if (mode == 0)
        return;
    if (interruptByRB1External()) {
        clearInterrupt_RB1External();
        if (INTCON2bits.INTEDG1 == 1) {
            enableTimer1(TIMER1_PRESCALE_8);
            TMR1 = 0;
            INTCON2bits.INTEDG1 = 0;
        } else {
            disableTimer1();
            float duration = getTimer1us(8);
            float distance = duration / 29.0 / 2.0 / 100;
            // serialPrintf("Distance: %.4f m\n", distance);

            if (distance < 0.3 && !turrning) {
                if (randomBinaryChoice()) {
                    motor_turn_left();
                } else {
                    motor_turn_right();
                }
                turrning = true;
            } else if (turrning && distance > 0.3) {
                __delay_ms(600);
                motor_stop();
                turrning = false;
            }
            INTCON2bits.INTEDG1 = 1;
        }
    }

    // 如果還有其他中斷如 Timer1 overflow, Timer3 overflow, ADConverter 等
    // if (interruptByTimer1Overflow()) {
    //     clearInterrupt_Timer1Overflow();
    // }
    // if (interruptByTimer3Overflow()) {
    //     clearInterrupt_Timer3Overflow();
    // }
    // if (interruptByADConverter()) {
    //     flash_state = getADConverter() / 341;
    //     clearInterrupt_ADConverter();
    // }
}

void __interrupt(low_priority) Lo_ISR(void) {
    if (processSerialReceive())
        return;
}

void main(void) {
    setIntrnalClock();
    setPortBPullup(PORTB_PULLUP_DISABLE);

    enableInterruptPriorityMode(1);  // enable the priority interrupt
    enableGlobalInterrupt(1);        // enable the global interrupt
    enablePeripheralInterrupt(1);    // enable peripheral interrupt
    // Button
    pinMode(PIN_RB0, PIN_INPUT);
    INTCON2bits.INTEDG0 = 1;
    enableInterrupt_RB0External();  // enable RB0 interrupt

    pinMode(PIN_RD6, PIN_OUTPUT);
    digitalWrite(PIN_RD6, 0);

    // sonic
    pinMode(PIN_RB1, PIN_INPUT);
    pinMode(PIN_RD4, PIN_OUTPUT);
    INTCON2bits.INTEDG1 = 1;
    enableInterrupt_RB1External(1);

    // UART
    serialBegin(9600, 0b0);
    serialOnReadLine = onReadLine;
    serialOnReadChar = onReadChar;

    // fan
    pinMode(PIN_RD5, PIN_OUTPUT);
    digitalWrite(PIN_RD5, 1);

    // digitalWrite(PIN_RA1, 0);
    // digitalWrite(PIN_RA2, 0);
    // digitalWrite(PIN_RA3, 0);

    pinMode(PIN_RD0, PIN_OUTPUT);
    pinMode(PIN_RD1, PIN_OUTPUT);
    pinMode(PIN_RD2, PIN_OUTPUT);
    pinMode(PIN_RD3, PIN_OUTPUT);

    digitalWrite(PIN_RD0, 1);
    digitalWrite(PIN_RD1, 1);
    digitalWrite(PIN_RD2, 1);
    digitalWrite(PIN_RD3, 1);

    // Servo initialize
    pinMode(PIN_RC2, PIN_OUTPUT);
    digitalWrite(PIN_RC2, 0);
    pinMode(PIN_RC1, PIN_OUTPUT);
    digitalWrite(PIN_RC1, 0);
    enableTimer2(TIMER2_PRESCALE_16, 0b0000);
    setTimer2InterruptPeriod(4100, 16, 1);
    setCCP1Mode(ECCP_MODE_PWM_HH);
    setCCP2Mode(ECCP_MODE_PWM_HH);

    int count = 0;
    serialPrint("Ready\n");
    char cache[20];

    while (1) {
        if (mode == 0) {
            continue;
        } else if (mode == 1) {
            TMR1 = 0;
            digitalWrite(PIN_RD4, 1);
            __delay_us(10);
            digitalWrite(PIN_RD4, 0);
            enableTimer1(TIMER1_PRESCALE_8);
            __delay_ms(50);
        } else if (mode == 2) {
            if (!turrning) {
                move_forward();
            }
            TMR1 = 0;
            digitalWrite(PIN_RD4, 1);
            __delay_us(10);
            digitalWrite(PIN_RD4, 0);
            enableTimer1(TIMER1_PRESCALE_8);
            __delay_ms(50);
        }
    }
    return;
}