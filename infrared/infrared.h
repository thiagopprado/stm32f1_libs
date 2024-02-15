/**
 * Author: thiagopereiraprado@gmail.com
 * 
 * @brief Infrared module.
 * 
 */
#ifndef INFRARED_H
#define INFRARED_H

#include <stdint.h>

#if !defined(INFRARED_PORT)
    #define INFRARED_PORT                   GPIOB
    #define INFRARED_GPIO_CLOCK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#endif

#if !defined(INFRARED_PIN)
    #define INFRARED_PIN        GPIO_PIN_6
#endif

#if !defined(INFRARED_TIMER)
    #define INFRARED_TIMER                  TIM4
    #define INFRARED_PWM_CLOCK_ENABLE()     __HAL_RCC_TIM4_CLK_ENABLE()
    #define INFRARED_TIMER_IRQ              TIM4_IRQHandler
#endif

#if !defined(INFRARED_IC_CH)
    #define INFRARED_IC_CH     TIM_CHANNEL_1
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