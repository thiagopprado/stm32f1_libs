/**
 * Author: thiagopereiraprado@gmail.com
 * 
 * @brief Infrared module implementation.
 * 
 */
#include "infrared.h"

#include <string.h>
#include <stdbool.h>

#include "stm32f1xx_hal.h"

#include "timer.h"

#define BIT_CHECK(value, bit)       ((value >> bit) & 0x01)
#define BIT_SET(value, bit)         (value |= 1 << bit)
#define BIT_CLEAR(value, bit)       (value &= ~(1 << bit))

/**
 * @brief State machine timings, for 100us base.
 * @{
 */
/** NEC */
#define IR_NEC_START_TIMEOUT        200
#define IR_NEC_BIT_BURST_TIMEOUT    10
#define IR_NEC_BIT_LOW_TIMEOUT      10
#define IR_NEC_BIT_TIMEOUT          30
#define IR_NEC_BIT_NR               32

/** RC6 */
#define IR_RC6_START_1_TIMEOUT      30
#define IR_RC6_START_2_TIMEOUT      15
#define IR_RC6_TOGGLE_1_TIMEOUT     30
#define IR_RC6_TOGGLE_2_TIMEOUT     10
#define IR_RC6_HALF_BIT_TIMEOUT     5
#define IR_RC6_BIT_TIMEOUT          10
#define IR_RC6_FIELD_BIT_NR         3
#define IR_RC6_BIT_NR               16
/** @} */

/**
 * @brief Remote control codes.
 * @{
 */
/** NEC */
#define IR_NEC_CODE_ENTER           0xE0E016E9
#define IR_NEC_CODE_ESC             0xE0E01AE5
#define IR_NEC_CODE_UP              0xE0E006F9
#define IR_NEC_CODE_DOWN            0xE0E08679
#define IR_NEC_CODE_LEFT            0xE0E0A659
#define IR_NEC_CODE_RIGHT           0xE0E046B9

/** RC6 */
#define IR_RC6_CODE_ENTER           0x3a00
#define IR_RC6_CODE_ESC             0x5000
#define IR_RC6_CODE_UP              0x1a00
#define IR_RC6_CODE_DOWN            0x9a00
#define IR_RC6_CODE_LEFT            0x5a00
#define IR_RC6_CODE_RIGHT           0xda00
/** @} */

typedef enum {
    IR_NEC_STATE_INIT = 0,
    IR_NEC_STATE_START,
    IR_NEC_STATE_READ_WAIT,
    IR_NEC_STATE_READ_GET,
    IR_NEC_STATE_STOP,
} infrared_nec_state_t;

typedef enum {
    IR_RC6_STATE_INIT = 0,
    IR_RC6_STATE_START_1,
    IR_RC6_STATE_START_2,
    IR_RC6_STATE_START_3,
    IR_RC6_STATE_READ_FIELD_WAIT,
    IR_RC6_STATE_READ_TOGGLE_1,
    IR_RC6_STATE_READ_TOGGLE_2,
    IR_RC6_STATE_READ_GET,
} infrared_rc6_state_t;

typedef struct {
    infrared_nec_state_t state;
    uint32_t value;
    uint32_t new_value;
    int8_t bit_idx;
    uint16_t last_timeshot;
} infrared_nec_ctrl_t;

typedef struct {
    infrared_rc6_state_t state;
    uint16_t value;
    uint16_t new_value;
    int8_t bit_idx;
    uint8_t field_bits_nr;
    bool transition_ctrl;
    uint16_t last_timeshot;
} infrared_rc6_ctrl_t;

static infrared_nec_ctrl_t nec_ctrl = { 0 };
static infrared_rc6_ctrl_t rc6_ctrl = { 0 };

static void infrared_nec_read(uint16_t timeshot);
static void infrared_rc6_read(uint16_t timeshot);
static void infrared_input_capture_callback(void);

/**
 * @brief NEC state machine.
 * 
 * @param timeshot  Timestamp of the pin change.
 */
static void infrared_nec_read(uint16_t timeshot) {
    uint16_t time_interval = timeshot - nec_ctrl.last_timeshot;
    bool pin_value = HAL_GPIO_ReadPin(INFRARED_PORT, INFRARED_PIN) == GPIO_PIN_SET;

    nec_ctrl.last_timeshot = timeshot;

    if (time_interval > 300) {
        nec_ctrl.state = IR_NEC_STATE_INIT;
    }

    switch (nec_ctrl.state) {
        case IR_NEC_STATE_INIT: {
            if (pin_value == false) {
                nec_ctrl.state = IR_NEC_STATE_START;
            }

            break;
        }
        case IR_NEC_STATE_START: {
            if (time_interval > IR_NEC_START_TIMEOUT) {
                nec_ctrl.state = IR_NEC_STATE_INIT;
            } else if (pin_value == false) {
                nec_ctrl.state = IR_NEC_STATE_READ_WAIT;
                nec_ctrl.bit_idx = IR_NEC_BIT_NR - 1;
                nec_ctrl.new_value = 0;
            }

            break;
        }
        case IR_NEC_STATE_READ_WAIT: {
            if (time_interval > IR_NEC_BIT_BURST_TIMEOUT) {
                // Invalid bit
                nec_ctrl.state = IR_NEC_STATE_INIT;
            } else if (pin_value == true) {
                nec_ctrl.state = IR_NEC_STATE_READ_GET;
            }

            break;
        }
        case IR_NEC_STATE_READ_GET: {
            if (time_interval > IR_NEC_BIT_TIMEOUT) {
                // Invalid bit
                nec_ctrl.state = IR_NEC_STATE_INIT;
            } else if (pin_value == false) {
                if (time_interval <= IR_NEC_BIT_LOW_TIMEOUT) {
                	BIT_CLEAR(nec_ctrl.new_value, nec_ctrl.bit_idx);
                } else {
                    BIT_SET(nec_ctrl.new_value, nec_ctrl.bit_idx);
                }

                nec_ctrl.bit_idx--;
                if (nec_ctrl.bit_idx < 0 ) {
                    nec_ctrl.bit_idx = IR_NEC_BIT_NR - 1;
                    nec_ctrl.state = IR_NEC_STATE_STOP;

                } else {
                    nec_ctrl.state = IR_NEC_STATE_READ_WAIT;
                }
            }

            break;
        }
        case IR_NEC_STATE_STOP: {
            if (time_interval > IR_NEC_BIT_BURST_TIMEOUT) {
                // Invalid bit
                nec_ctrl.state = IR_NEC_STATE_INIT;
            } else if (pin_value == true) {
                nec_ctrl.state = IR_NEC_STATE_INIT;
                nec_ctrl.value = nec_ctrl.new_value;
            }

            break;
        }
        default: {
            break;
        }
    }
}

/**
 * @brief RC6 state machine.
 * 
 * @param timeshot  Timestamp of the pin change.
 */
static void infrared_rc6_read(uint16_t timeshot) {
    uint16_t time_interval = timeshot - rc6_ctrl.last_timeshot;
    bool pin_value = HAL_GPIO_ReadPin(INFRARED_PORT, INFRARED_PIN) == GPIO_PIN_SET;

    rc6_ctrl.last_timeshot = timeshot;

    if (time_interval > 100 && rc6_ctrl.state != IR_RC6_STATE_INIT) {
        rc6_ctrl.state = IR_RC6_STATE_INIT;
    }

    switch (rc6_ctrl.state) {
        case IR_RC6_STATE_INIT: {
            if (pin_value == false) {
                rc6_ctrl.state = IR_RC6_STATE_START_1;
            }

            break;
        }
        case IR_RC6_STATE_START_1: {
            if (time_interval > IR_RC6_START_1_TIMEOUT) {
                // Invalid start
                rc6_ctrl.state = IR_RC6_STATE_INIT;
            } else if (pin_value == true) {
                rc6_ctrl.state = IR_RC6_STATE_START_2;
            }

            break;
        }
        case IR_RC6_STATE_START_2: {
            if (time_interval > IR_RC6_START_2_TIMEOUT) {
                // Invalid start
                rc6_ctrl.state = IR_RC6_STATE_INIT;
            } else if (pin_value == false) {
                rc6_ctrl.state = IR_RC6_STATE_START_3;
            }

            break;
        }
        case IR_RC6_STATE_START_3: {
            if (time_interval > IR_RC6_HALF_BIT_TIMEOUT) {
                // Invalid start
                rc6_ctrl.state = IR_RC6_STATE_INIT;
            } else if (pin_value == true) {
                rc6_ctrl.state = IR_RC6_STATE_READ_FIELD_WAIT;
                rc6_ctrl.transition_ctrl = false;
            }

            break;
        }
        case IR_RC6_STATE_READ_FIELD_WAIT: {
            if (time_interval > IR_RC6_BIT_TIMEOUT) {
                // Invalid field
                rc6_ctrl.state = IR_RC6_STATE_INIT;
                rc6_ctrl.field_bits_nr = 0;

            } else if (time_interval > IR_RC6_HALF_BIT_TIMEOUT || rc6_ctrl.transition_ctrl == true) {
                rc6_ctrl.field_bits_nr++;
                if (rc6_ctrl.field_bits_nr == IR_RC6_FIELD_BIT_NR) {
                    rc6_ctrl.field_bits_nr = 0;
                    rc6_ctrl.state = IR_RC6_STATE_READ_TOGGLE_1;
                }
                rc6_ctrl.transition_ctrl = false;

            } else if (rc6_ctrl.transition_ctrl == false) {
                rc6_ctrl.transition_ctrl = true;
            }

            break;
        }
        case IR_RC6_STATE_READ_TOGGLE_1: {
            if (time_interval > IR_RC6_TOGGLE_1_TIMEOUT) {
                // Invalid toggle
                rc6_ctrl.state = IR_RC6_STATE_INIT;
                rc6_ctrl.transition_ctrl = false;
            } else if (time_interval <= IR_RC6_HALF_BIT_TIMEOUT && rc6_ctrl.transition_ctrl == false) {
                rc6_ctrl.transition_ctrl = true;
            } else {
                rc6_ctrl.state = IR_RC6_STATE_READ_TOGGLE_2;
            }

            break;
        }
        case IR_RC6_STATE_READ_TOGGLE_2: {
            rc6_ctrl.state = IR_RC6_STATE_READ_GET;
            rc6_ctrl.bit_idx = 0;
            rc6_ctrl.new_value = 0;
            if (time_interval < IR_RC6_TOGGLE_2_TIMEOUT) {
                break;
            } else {
                time_interval = IR_RC6_BIT_TIMEOUT;
            }
        }
        case IR_RC6_STATE_READ_GET: {
            if (time_interval > IR_RC6_BIT_TIMEOUT) {
                // Invalid bit
                rc6_ctrl.state = IR_RC6_STATE_INIT;
                rc6_ctrl.transition_ctrl = false;
            } else if (time_interval <= IR_RC6_HALF_BIT_TIMEOUT && rc6_ctrl.transition_ctrl == false && rc6_ctrl.bit_idx != 0) {
                rc6_ctrl.transition_ctrl = true;
            } else {
                if (pin_value == true) {
                    BIT_SET(rc6_ctrl.new_value, rc6_ctrl.bit_idx);
                }

                rc6_ctrl.bit_idx++;
                if (rc6_ctrl.bit_idx == IR_RC6_BIT_NR) {
                    rc6_ctrl.bit_idx = 0;
                    rc6_ctrl.state = IR_RC6_STATE_INIT;
                    rc6_ctrl.value = rc6_ctrl.new_value;
                }

                rc6_ctrl.transition_ctrl = false;
            }

            break;
        }
        default: {
            break;
        }
    }
}

/**
 * @brief Input Capture callback.
 */
static void infrared_input_capture_callback(void) {
    uint16_t ic_timeshot = timer_get_input_capture_counter(INFRARED_TIMER, INFRARED_IC_CH);

    timer_invert_input_capture_polarity(INFRARED_TIMER, INFRARED_IC_CH);
    infrared_nec_read(ic_timeshot);
    infrared_rc6_read(ic_timeshot);
}

/**
 * @brief Sets up infrared pin and timer.
 */
void infrared_setup(void) {
    INFRARED_GPIO_CLOCK_ENABLE();

    GPIO_InitTypeDef gpio_init = {
        .Pin = INFRARED_PIN,
        .Mode = GPIO_MODE_AF_INPUT,
        .Pull = GPIO_NOPULL,
        .Speed = GPIO_SPEED_FREQ_HIGH,
    };
    HAL_GPIO_Init(INFRARED_PORT, &gpio_init);

    timer_setup(INFRARED_TIMER, 7199, 0xFFFF);
    timer_input_capture_setup(INFRARED_TIMER, INFRARED_IC_CH);
    timer_invert_input_capture_polarity(INFRARED_TIMER, INFRARED_IC_CH);
    timer_attach_input_capture_callback(INFRARED_TIMER, INFRARED_IC_CH, infrared_input_capture_callback);
}

/**
 * @brief Gets the value read for NEC protocol.
 * 
 * @return Value read.
 */
uint32_t infrared_read_nec(void)
{
    uint32_t read_value = nec_ctrl.value;

    nec_ctrl.value = 0;

    return read_value;
}

/**
 * @brief Gets the value read for RC6 protocol.
 * 
 * @return Value read.
 */
uint32_t infrared_read_rc6(void)
{
    uint16_t read_value = rc6_ctrl.value;

    rc6_ctrl.value = 0;

    return read_value;
}

/**
 * @brief Decodes NEC and RC6 read values into a key code.
 * 
 * @return Key pressed.
 */
ir_key_id_t infrared_decode(void) {
    ir_key_id_t key_pressed = INFRARED_KEY_NONE;
    uint32_t nec_read = infrared_read_nec();
    uint16_t rc6_read = infrared_read_rc6();

    if (nec_read == IR_NEC_CODE_ENTER || rc6_read == IR_RC6_CODE_ENTER) {
        key_pressed = INFRARED_KEY_ENTER;
    } else if (nec_read == IR_NEC_CODE_ESC || rc6_read == IR_RC6_CODE_ESC) {
        key_pressed = INFRARED_KEY_ESC;
    } else if (nec_read == IR_NEC_CODE_UP || rc6_read == IR_RC6_CODE_UP) {
        key_pressed = INFRARED_KEY_UP;
    } else if (nec_read == IR_NEC_CODE_DOWN || rc6_read == IR_RC6_CODE_DOWN) {
        key_pressed = INFRARED_KEY_DOWN;
    } else if (nec_read == IR_NEC_CODE_LEFT || rc6_read == IR_RC6_CODE_LEFT) {
        key_pressed = INFRARED_KEY_LEFT;
    } else if (nec_read == IR_NEC_CODE_RIGHT || rc6_read == IR_RC6_CODE_RIGHT) {
        key_pressed = INFRARED_KEY_RIGHT;
    }

    return key_pressed;
}
