/**
 * Author: thiagopereiraprado@gmail.com
 * 
 * @brief Buzzer module.
 * 
 */
#ifndef BUZZER_H
#define BUZZER_H

#if !defined(BUZZER_PORT)
    #define BUZZER_PORT                 GPIOA
    #define BUZZER_GPIO_CLOCK_ENABLE()  __HAL_RCC_GPIOA_CLK_ENABLE()
#endif

#if !defined(BUZZER_PIN)
    #define BUZZER_PIN      GPIO_PIN_3
#endif

#if !defined(BUZZER_TIMER)
    #define BUZZER_TIMER    TIMER_2
#endif

#if !defined(BUZZER_PWM_CH)
    #define BUZZER_PWM_CH   TIMER_CH_4
#endif

typedef enum {
    BUZZER_NOTE_C3 = 0,
    BUZZER_NOTE_D3,
    BUZZER_NOTE_E3,
    BUZZER_NOTE_F3,
    BUZZER_NOTE_G3,
    BUZZER_NOTE_A3,
    BUZZER_NOTE_B3,
    BUZZER_NOTE_C4,
    BUZZER_NOTE_D4,
    BUZZER_NOTE_E4,
    BUZZER_NOTE_F4,
    BUZZER_NOTE_G4,
    BUZZER_NOTE_A4,
    BUZZER_NOTE_B4,
    BUZZER_NOTE_ST,
    BUZZER_NOTE_NR,
} buzzer_note_t;

void buzzer_setup(void);
void buzzer_play_note(buzzer_note_t note);

#endif /* BUZZER_H */