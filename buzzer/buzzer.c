/**
 * Author: thiagopereiraprado@gmail.com
 * 
 * @brief Buzzer module implementation.
 * 
 */
#include "buzzer.h"

/**
 * @brief Timer prescaler for buzzer pwm.
 * 
 * @note ARR will be modified to generate the frequencies.
 */
#define BUZZER_PWM_PSC  71
#define BUZZER_PWM_ARR  999

/**
 * @brief Buzzer main frequency.
 * 
 * Frequency generated after the prescaler (without ARR).
 */
#define BUZZER_PWM_MAIN_FREQ  1000000

static uint32_t buzzer_note_freq[BUZZER_NOTE_NR] = { 131, 147, 165, 175, 196, 220, 247, 262, 294, 330, 349, 392, 440, 494, 0 };

/**
 * @brief Sets up buzzer pin and peripherals.
 */
void buzzer_setup(void) {
    gpio_setup(BUZZER_PORT, BUZZER_PIN, GPIO_MODE_OUTPUT_50, GPIO_CFG_OUT_AF_PUSH_PULL);

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
