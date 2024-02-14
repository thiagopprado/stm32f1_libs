/**
 * Author: thiagopereiraprado@gmail.com
 * 
 * @brief Buzzer module implementation.
 * 
 */
/* Includes ------------------------------------------------------------------*/
#include "buzzer.h"

#include "stm32f1xx_hal.h"

/* Private types -------------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
/**
 * @brief Buzzer main frequency.
 * 
 * Frequency generated after the prescaler (without ARR).
 */
#define BUZZER_PWM_MAIN_FREQ    1000000

/**
 * @brief Timer prescaler for buzzer pwm.
 * 
 * @note ARR will be modified to generate the frequencies.
 */
#define BUZZER_PWM_PSC          ((72000000 / BUZZER_PWM_MAIN_FREQ) - 1)

/* Private variables ---------------------------------------------------------*/
static const uint32_t buzzer_note_freq[BUZZER_NOTE_NR] = { 131, 147, 165, 175, 196, 220, 247, 262, 294, 330, 349, 392, 440, 494, 0 };
static TIM_HandleTypeDef timer_handle = { 0 };

/* Private function prototypes -----------------------------------------------*/

/* Private function implementation--------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
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

    BUZZER_PWM_CLOCK_ENABLE();

    timer_handle.Instance = BUZZER_TIMER;
    timer_handle.Init.Prescaler = BUZZER_PWM_PSC;
    timer_handle.Init.Period = 0;
    timer_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&timer_handle);

    TIM_OC_InitTypeDef pwm_config = { 0 };
    pwm_config.OCMode = TIM_OCMODE_PWM1;
    HAL_TIM_PWM_ConfigChannel(&timer_handle, &pwm_config, BUZZER_PWM_CH);
    HAL_TIM_PWM_Start(&timer_handle, BUZZER_PWM_CH);
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
        __HAL_TIM_SET_COMPARE(&timer_handle, BUZZER_PWM_CH, 0);
    } else {
        uint32_t new_arr = BUZZER_PWM_MAIN_FREQ / buzzer_note_freq[note];

        __HAL_TIM_SET_AUTORELOAD(&timer_handle, new_arr - 1);
        __HAL_TIM_SET_COMPARE(&timer_handle, BUZZER_PWM_CH, new_arr / 2);
    }
}
