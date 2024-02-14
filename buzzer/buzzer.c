/**
 * Author: thiagopereiraprado@gmail.com
 * 
 * @brief Buzzer module implementation.
 * 
 */
#include "buzzer.h"

#include "timer.h"

#include "stm32f1xx_hal.h"

/**
 * @brief Timer prescaler for buzzer pwm.
 * 
 * @note ARR will be modified to generate the frequencies.
 */

/**
 * @brief Buzzer main frequency.
 * 
 * Frequency generated after the prescaler (without ARR).
 */
#define BUZZER_PWM_MAIN_FREQ    1000000
#define BUZZER_PWM_PSC          ((72000000 / BUZZER_PWM_MAIN_FREQ) - 1)
#define BUZZER_PWM_ARR          999

static uint32_t buzzer_note_freq[BUZZER_NOTE_NR] = { 131, 147, 165, 175, 196, 220, 247, 262, 294, 330, 349, 392, 440, 494, 0 };

/**
 * @brief Sets up buzzer pin and peripherals.
 */
void buzzer_setup(void) {
    BUZZER_GPIO_CLOCK_ENABLE();

    GPIO_InitTypeDef gpio_init = {
        .Pin = BUZZER_PIN,
        .Mode = GPIO_MODE_AF_PP,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    HAL_GPIO_Init(BUZZER_PORT, &gpio_init);

    timer_setup(BUZZER_TIMER, BUZZER_PWM_PSC, BUZZER_PWM_ARR);
    timer_pwm_setup(BUZZER_TIMER, BUZZER_PWM_CH);
}

/**
 * @brief Update the pwm channel with the note frequency.
 * 
 * @param note  Note to be played.
 */
void buzzer_play_note(buzzer_note_t note) {
    if (note >= BUZZER_NOTE_NR) {
        return;
    }

    if (note == BUZZER_NOTE_ST) {
        timer_pwm_set_duty(BUZZER_TIMER, BUZZER_PWM_CH, 0);
    } else {
        uint32_t new_arr = BUZZER_PWM_MAIN_FREQ / buzzer_note_freq[note];

        timer_update_psc(BUZZER_TIMER, BUZZER_PWM_PSC, new_arr - 1);
        timer_pwm_set_duty(BUZZER_TIMER, BUZZER_PWM_CH, new_arr / 2);
    }
}
