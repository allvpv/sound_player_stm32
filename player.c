#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stm32.h>
#include <gpio.h>

#define PLAYBACK_STEP 75680 /* About 0.4 of second */
_Static_assert(PLAYBACK_STEP % 11 == 0, "Playback adjustment step: must a be multiple of 11, "
    "otherwise pointer to bit packed data will end up in wrong state.");

const unsigned char sound1[] = {
/* #embed "sound_funny.raw11" soon */
#include "sound_funny.raw11.cdata"
};

void SetupForPeriodicQuery_Tim2(void);
void SetupForPWM_Tim3_PB5(void);

void DisableOutput(void);
void EnableOutput(void);

void MoveSongBackward(void);
void MoveSongForward(void);
void ResetSongPosition(void);
void SetSongPlayback(void);
void SetupSongPlayback(const unsigned char* song_data, size_t song_data_size);

bool FetchNextSongSample(uint16_t* write_to);

#define TIM_PWM_BASE_CLOCK 96'000'000

void Setup96MhzClock(void) {
    int m = 8;
    int n = 384;
    int p = 4;
    int q = 8;

    // Turn on flash cache (3WS latency: 90MHz  < HCLK < 100MHz)
    FLASH->ACR = FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_3WS;

    // Disable PLL, HSE
    RCC->CR &= ~(RCC_CR_PLLI2SON | RCC_CR_PLLON | RCC_CR_HSEBYP | RCC_CR_HSEON);

    // Turn on HSE
    RCC->CR |= RCC_CR_HSEON;

    // Wait for the osciilator to warm up
    while (!(RCC->CR & RCC_CR_HSERDY))
        __NOP();

    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    PWR->CR |= PWR_CR_VOS;

    uint32_t reg = RCC->CFGR;

    // Set divider for fHCLK
    reg &= ~RCC_CFGR_HPRE;
    reg |= RCC_CFGR_HPRE_DIV1;

    // Set divider for fPCLK1
    reg &= ~RCC_CFGR_PPRE1;
    reg |= RCC_CFGR_PPRE1_DIV2;

    // Set divider for fPCLK2
    reg &= ~RCC_CFGR_PPRE2;
    reg |= RCC_CFGR_PPRE2_DIV1;

    // Set new dividers
    RCC->CFGR = reg;

    // Compute value in PLLCFGR from bits
    RCC->PLLCFGR = m | (n << 6) | (((p >> 1) - 1) << 16) | RCC_PLLCFGR_PLLSRC_HSE | (q << 24);

    // Turn on loop oscillator
    RCC->CR |= RCC_CR_PLLON;

    // Wait for warm-up to complete.
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
        __NOP();

    // Set fSYSCLK = fPLL OUT
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= RCC_CFGR_SW_PLL;

    // Wait
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL)
        __NOP();
}

void SetupForPWM_Tim3_PB5(void) {
    /* Enable TIM3 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* Disable prescaller (only useful when timing long events) */
    TIM3->PSC = 0;

    /* When ARPE is enabled, ARR value is buffered in the Shadow Register, so
     * changing the ARR value will not have any effect on the current cycle
     * until the UEV happens */
    TIM3->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;

    /* Enable update interrupt */
    TIM3->DIER |= TIM_DIER_UIE;

    /* Temporary disable NVIC interrupt and set priority to 0. */
    NVIC_DisableIRQ(TIM3_IRQn);
    NVIC_SetPriority(TIM3_IRQn, 0);

    /* Set mode for PC7: alternative function */
    GPIOC->MODER |= GPIO_MODER_MODER7_1;

    /* Set alternative function for output pin (TIM3_CH2 for PC_7 - AF_2) */
    GPIOC->AFR[0] |= GPIO_AFRL_AFSEL7_1;

    /* Disable no PU_PD for PC_7 */
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPD7;

    /* Disable output */
    DisableOutput();

    /* Set frequency to 44.1Khz */
    TIM3->ARR = TIM_PWM_BASE_CLOCK / 44'100;

    /* Set PWM1 function on CH2 */
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;

    /* Start timer; UG: set counter to 0 when overflow */
    TIM3->EGR |= TIM_EGR_UG;
}

#define ENABLE_TIM2_COUNTER()           \
    do {                                \
        /* Enable counter. */           \
        TIM2->CR1 |= TIM_CR1_CEN;       \
        /* Enable NVIC interrupt. */    \
        NVIC_EnableIRQ(TIM2_IRQn);      \
    } while (false);

#define DISABLE_TIM2_COUNTER()          \
    do {                                \
        /* Disable counter. */          \
        TIM2->CR1 &= ~TIM_CR1_CEN;      \
        /* Disable NVIC interrupt. */   \
        NVIC_DisableIRQ(TIM2_IRQn);     \
    } while (false);

void SetupForPeriodicQuery_Tim2(void) {
    /* Enable TIM2 clock. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Set timer to 10ms */
    TIM2->ARR = 500;
    TIM2->PSC = TIM_PWM_BASE_CLOCK / 50'000;

    TIM2->CR1 &= ~TIM_CR1_CKD;  /* No divider. */
    TIM2->CR1 &= ~TIM_CR1_ARPE; /* Disable preload. */
    TIM2->CR1 &= ~TIM_CR1_OPM;  /* Continous mode. Enable timer to run continously. */
    TIM2->CR1 &= ~TIM_CR1_CMS;  /* Set CMS to alignment edge. */
    TIM2->CR1 &= ~TIM_CR1_DIR;  /* Set DIR to counting up. */
    TIM2->CR1 |= TIM_CR1_CEN;   /* Enable Counter. */

    /* Enable interrupt in DIER. */
    TIM2->DIER |= TIM_DIER_CC1IE;

    /* Disable NVIC interrupt and counter. */
    DISABLE_TIM2_COUNTER();

    /* Set maximal priority for the interrupt. */
    NVIC_SetPriority(TIM3_IRQn, 0);
}

#define RED_LED_GPIO GPIOA
#define GREEN_LED_GPIO GPIOA

#define RED_LED_PIN 6
#define GREEN_LED_PIN 7

#define LEFT_JOY_BTN_PIN   3
#define RIGHT_JOY_BTN_PIN  4
#define DOWN_JOY_BTN_PIN   6
#define PRESS_JOY_BTN_PIN  10

#define GET_BUTTON_LEFT()  GET_BUTTON(GPIOB, LEFT_JOY_BTN_PIN)
#define GET_BUTTON_RIGHT() GET_BUTTON(GPIOB, RIGHT_JOY_BTN_PIN)
#define GET_BUTTON_DOWN()  GET_BUTTON(GPIOB, DOWN_JOY_BTN_PIN)
#define GET_BUTTON_PRESS() GET_BUTTON(GPIOB, PRESS_JOY_BTN_PIN)

/* Button value fetching */
#define GET_BUTTON(GPIO, PIN) (!(((GPIO)->IDR >> (PIN)) & 1))

#define LED_ON(LED_GPIO, LED_PIN)                   \
    (LED_GPIO)->BSRR = 1 << ((LED_PIN) + 16)

#define LED_OFF(LED_GPIO, LED_PIN)                  \
    (LED_GPIO)->BSRR = 1 << (LED_PIN)

#define IS_LED_ON(LED_GPIO, LED_PIN)                \
    (!(((LED_GPIO)->IDR >> (LED_PIN)) & 1))

#define LED_TOGGLE(LED_GPIO, LED_PIN)               \
    if (IS_LED_ON((LED_GPIO), (LED_PIN))) {         \
        LED_OFF((LED_GPIO), (LED_PIN));             \
    } else {                                        \
        LED_ON((LED_GPIO), (LED_PIN));              \
    }

#define CONFIGURE_LED(LED_GPIO, LED_PIN)            \
    GPIOoutConfigure((LED_GPIO),                    \
                     (LED_PIN),                     \
                     GPIO_OType_PP,                 \
                     GPIO_Low_Speed,                \
                     GPIO_PuPd_NOPULL)

#define CONFIGURE_BUTTON(BTN_GPIO, BTN_PIN)         \
    GPIOinConfigure((BTN_GPIO),                     \
                    (BTN_PIN),                      \
                    GPIO_PuPd_NOPULL,               \
                    EXTI_Mode_Interrupt,            \
                    EXTI_Trigger_Falling)

int main(void) {
    Setup96MhzClock();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN |
                    RCC_AHB1ENR_GPIOCEN;

    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    LED_OFF(RED_LED_GPIO, RED_LED_PIN);
    LED_OFF(GREEN_LED_GPIO, GREEN_LED_PIN);

    CONFIGURE_LED(GREEN_LED_GPIO, GREEN_LED_PIN);
    CONFIGURE_LED(RED_LED_GPIO, RED_LED_PIN);
    CONFIGURE_BUTTON(GPIOB, LEFT_JOY_BTN_PIN);
    CONFIGURE_BUTTON(GPIOB, RIGHT_JOY_BTN_PIN);
    CONFIGURE_BUTTON(GPIOB, DOWN_JOY_BTN_PIN);
    CONFIGURE_BUTTON(GPIOB, PRESS_JOY_BTN_PIN);

    EXTI->PR |= 0;
    NVIC_EnableIRQ(EXTI3_IRQn);     /* LEFT joystick button  */
    NVIC_EnableIRQ(EXTI4_IRQn);     /* RIGHT joystick button */
    NVIC_EnableIRQ(EXTI9_5_IRQn);   /* DOWN joystick button  */
    NVIC_EnableIRQ(EXTI15_10_IRQn); /* PRESS joystick button */

    SetupForPWM_Tim3_PB5();
    SetupForPeriodicQuery_Tim2();

    /* Play song */
    SetupSongPlayback(sound1, sizeof(sound1) / sizeof(*sound1));

    while (true);
}

/*
 * Setting up song
 */
void DisableOutput(void) {
    /* Disable output on CH2 */
    TIM3->CCER &= ~0x0010;
}

void EnableOutput(void) {
    /* Enable output on CH2 */
    TIM3->CCER |= 0x0010;
}

static bool gSongPlayback = false;
static const unsigned char* gSongData = NULL;
static uint32_t gSongDataSize = 0;
static uint32_t gSongIndexByte = 0;
static uint32_t gSongIndexBitFromMsb = 0;

void MoveSongBackward(void) {
    if (gSongIndexByte >= PLAYBACK_STEP)
        gSongIndexByte -= PLAYBACK_STEP;
    else {
        gSongIndexByte = 0;
        gSongIndexBitFromMsb = 0;
    }
}

void MoveSongForward(void) {
    if (gSongIndexByte < gSongDataSize)
        gSongIndexByte += PLAYBACK_STEP;
}

void ResetSongPosition(void) {
    gSongIndexByte = 0;
    gSongIndexBitFromMsb = 0;
}

void SetSongPlayback(void) {
    if (gSongPlayback) {
        EnableOutput();
        NVIC_EnableIRQ(TIM3_IRQn);
        LED_TOGGLE(GREEN_LED_GPIO, GREEN_LED_PIN);
    } else {
        DisableOutput();
        NVIC_DisableIRQ(TIM3_IRQn);
        LED_TOGGLE(GREEN_LED_GPIO, GREEN_LED_PIN);
    }
}

void SetupSongPlayback(const unsigned char* song_data, size_t song_data_size) {
    gSongData = song_data;
    gSongDataSize = song_data_size;
    gSongPlayback = true;

    ResetSongPosition();
    SetSongPlayback();
}

/*
 * Playing song
 */

/* Unpack bit-packed data */
bool FetchNextSongSample(uint16_t* write_to) {
    if (gSongIndexByte + 2 >= gSongDataSize)
        return false;

    uint16_t sample = *(uint16_t*) &gSongData[gSongIndexByte];

    sample = (sample >> 8) | (sample << 8);
    sample <<= gSongIndexBitFromMsb;
    sample >>= 5;

    if (gSongIndexBitFromMsb < 5) {
        gSongIndexBitFromMsb += 11;

    } else if (gSongIndexBitFromMsb == 5) {
        gSongIndexByte += 2;
        gSongIndexBitFromMsb = 0;

    } else {
        uint32_t shift = gSongIndexBitFromMsb - 5;

        gSongIndexByte += 2;
        gSongIndexBitFromMsb = shift;

        if (gSongIndexByte >= gSongDataSize)
            return false;

        uint16_t sample_part2 = *(uint16_t*) &gSongData[gSongIndexByte];

        sample_part2 = (sample_part2 >> 8) | (sample_part2 << 8);
        sample_part2 >>= (16 - shift);

        sample |= sample_part2;
    }

    *write_to = sample;
    return true;
}

/* Load next sample */
void TIM3_IRQHandler(void) {
    if (!(TIM3->SR & TIM3->DIER & TIM_SR_UIF))
        return;

    TIM3->SR = ~TIM_SR_UIF;

    // TODO: stop timer when playback is off

    if (!gSongPlayback)
        return;

    uint16_t next_sample;

    if (!FetchNextSongSample(&next_sample)) {
        gSongPlayback = false;
        SetSongPlayback();
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

/* Every 10 ms when counter is enabled */
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
            SetSongPlayback();
        }

        if (idle) {
            DISABLE_TIM2_COUNTER();
            LED_OFF(RED_LED_GPIO, RED_LED_PIN);
        }
    }
}

void maybe_handle(struct button_state* state, int pin, bool is_pressed) {
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
