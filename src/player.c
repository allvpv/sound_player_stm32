#include "configuration.h"
#include "song_data.h"

const unsigned char sound1[] = {
#include "sound_jingle.raw11.cdata"
};

static void DisableOutput(void) {
    /* Disable output on CH2 */
    TIM3->CCER &= ~0x0010;
    NVIC_DisableIRQ(TIM3_IRQn);
    TIM3->CR1 &= ~TIM_CR1_CEN;
}

static void EnableOutput(void) {
    /* Enable output on CH2 */
    TIM3->CCER |= 0x0010;
    NVIC_EnableIRQ(TIM3_IRQn);
    TIM3->CR1 |= TIM_CR1_CEN;
}

static bool gSongPlayback = false;

static void ApplySongPlayback(void) {
    if (gSongPlayback) {
        EnableOutput();
        LED_TOGGLE(GREEN_LED_GPIO, GREEN_LED_PIN);
    } else {
        DisableOutput();
        LED_TOGGLE(GREEN_LED_GPIO, GREEN_LED_PIN);
    }
}

/* Load next sample */
void TIM3_IRQHandler(void) {
    if (!(TIM3->SR & TIM3->DIER & TIM_SR_UIF))
        return;

    TIM3->SR = ~TIM_SR_UIF;

    if (!gSongPlayback)
        return;

    uint16_t next_sample;

    if (!FetchNextSongSample(&next_sample)) {
        gSongPlayback = false;
        ApplySongPlayback();
        return;
    }

    TIM3->CCR2 = next_sample;
}

/* Debouncing parameters */
#define WARMUP_STEPS 4
#define COOLOUT_STEPS 8

/*
 * Song position control
 */
struct button_state {
    union {
        uint16_t warmup_steps_left;
        uint16_t coolout_steps_left;
    };

    enum {
        IDLE = 0, WARMING_UP, COOLING_OUT
    } currently;
};

static struct button_state gLeftState = {};
static struct button_state gRightState = {};
static struct button_state gDownState = {};
static struct button_state gPressState = {};

typedef enum {
    WAITING,
    PRESSED,
    IS_IDLE,
} CheckButtonResult;

static CheckButtonResult CheckButton(bool now_pressed, struct button_state* state) {
    if (state->currently == WARMING_UP) {
        if (!now_pressed) {
            state->currently = IDLE;
            state->warmup_steps_left = 0;
            state->coolout_steps_left = 0;

            return IS_IDLE;
        }

        if (--state->warmup_steps_left == 0) {
            state->currently = COOLING_OUT;
            state->coolout_steps_left = COOLOUT_STEPS;

            return PRESSED;
        }

        return WAITING;
    }

    else if (state->currently == COOLING_OUT) {
        if (--state->coolout_steps_left == 0) {
            state->currently = IDLE;
            return IS_IDLE;
        }

        return WAITING;
    }

    return IS_IDLE;
}

/* Every 10 ms (when enabled) */
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM2->DIER & TIM_SR_CC1IF) {
        TIM2->SR &= ~TIM_SR_CC1IF;

        LED_ON(RED_LED_GPIO, RED_LED_PIN);

        CheckButtonResult result;
        bool idle = true;

        result = CheckButton(GET_BUTTON_LEFT(), &gLeftState);
        idle = idle && (result == IS_IDLE);

        if (result == PRESSED)
            MoveSongBackward();

        result = CheckButton(GET_BUTTON_RIGHT(), &gRightState);
        idle = idle && (result == IS_IDLE);

        if (result == PRESSED)
            MoveSongForward();

        result = CheckButton(GET_BUTTON_DOWN(), &gDownState);
        idle = idle && (result == IS_IDLE);

        if (result == PRESSED)
            ResetSongPosition();

        result = CheckButton(GET_BUTTON_PRESS(), &gPressState);
        idle = idle && (result == IS_IDLE);

        if (result == PRESSED) {
            gSongPlayback ^= true;
            ApplySongPlayback();
        }

        if (idle) {
            DISABLE_TIM2_COUNTER();
            LED_OFF(RED_LED_GPIO, RED_LED_PIN);
        }
    }
}

static void maybe_handle(struct button_state* state, int pin, bool is_pressed) {
    if (EXTI->PR & (1 << pin)) {
        EXTI->PR = 1 << pin;

        if (is_pressed && state->currently == IDLE) {
            state->currently = WARMING_UP;
            state->warmup_steps_left = WARMUP_STEPS;
            ENABLE_TIM2_COUNTER();
        }
    }
}

void EXTI3_IRQHandler(void) {
    maybe_handle(&gLeftState, LEFT_JOY_BTN_PIN, GET_BUTTON_LEFT());
}

void EXTI4_IRQHandler(void) {
    maybe_handle(&gRightState, RIGHT_JOY_BTN_PIN, GET_BUTTON_RIGHT());
}

void EXTI9_5_IRQHandler(void) {
    maybe_handle(&gDownState, DOWN_JOY_BTN_PIN, GET_BUTTON_DOWN());
}

void EXTI15_10_IRQHandler(void) {
    maybe_handle(&gPressState, PRESS_JOY_BTN_PIN, GET_BUTTON_PRESS());
}


int main(void) {
    Setup96MhzClock();
    SetupLedButtonsInterrupts();
    SetupForPWM_Tim3_PC7();
    SetupForPeriodicQuery_Tim2();

    /* Play song */
    SetupSongPlayback(sound1, sizeof(sound1) / sizeof(*sound1));
    gSongPlayback = true;
    ApplySongPlayback();

    __WFI();
}
