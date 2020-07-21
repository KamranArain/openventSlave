#ifndef __HEADER_H
#define __HEADER_H

enum STATES
{
    RUNNING_FWD = 1,
    STALL,
    HOMING,
    HOME,
    RUNNING_REV
};

/*********Pinout***********/
                           //| Arduino | STM32F1xx |
                           //|---------------------|
#define LS1_PIN 6          //|   6     |    PB_3   |
#define LS2_PIN 5          //|   5     |    PB_4   |
#define EN1_PIN 2          //|   2     |    PB_7   |
#define EN2_PIN 3          //|   3     |    PB_6   |
#ifdef __AVR__             //|---------|-----------|
#define DIR_PIN 10         //|   10    |    ----   |
#define PWM_OUT 9          //|   9     |    ----   |
#elif defined(STM32F1xx)   //|---------|-----------|
#define DIR_PIN 12         //|  ----   |    PA_8   |
#define PWM_OUT 20         //|  ----   |    PA_0   |
#endif                     //|_____________________|

#endif
