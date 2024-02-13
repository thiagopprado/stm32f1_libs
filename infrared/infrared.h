/**
 * Author: thiagopereiraprado@gmail.com
 * 
 * @brief Infrared module.
 * 
 */
#ifndef INFRARED_H
#define INFRARED_H

#include "gpio.h"

#include <stdint.h>

#include "timer.h"

#if !defined(INFRARED_PORT)
    #define INFRARED_PORT     GPIO_PORTB
#endif

#if !defined(INFRARED_PIN)
    #define INFRARED_PIN      6
#endif

#if !defined(INFRARED_TIMER)
    #define INFRARED_TIMER     TIMER_4
#endif

#if !defined(INFRARED_IC_CH)
    #define INFRARED_IC_CH     TIMER_CH_1
#endif

typedef enum {
    INFRARED_KEY_NONE = 0,
    INFRARED_KEY_ENTER,
    INFRARED_KEY_ESC,
    INFRARED_KEY_UP,
    INFRARED_KEY_DOWN,
    INFRARED_KEY_LEFT,
    INFRARED_KEY_RIGHT,
} ir_key_id_t;

void infrared_setup(void);
uint32_t infrared_read_nec(void);
uint32_t infrared_read_rc6(void);
ir_key_id_t infrared_decode(void);

#endif /* INFRARED_H */