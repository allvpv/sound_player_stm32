#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <stm32.h>
#include <gpio.h>

#define SET_PLAYBACK_LED_INDICATOR() \
    GPIOA->BSRR = 1 << (6 + 16)

#define CLEAR_PLAYBACK_LED_INDICATOR() \
    GPIOA->BSRR = 1 << 6

/* Debouncing parameters */
#define WARMUP_COUNT 4
#define COOLOUT_COUNT 50

#define PLAYBACK_STEP 75680 /* About 0.4 of second */
_Static_assert(PLAYBACK_STEP % 11 == 0, "Playback adjustment step: must a be multiple of 11, "
    "otherwise pointer to bit packed data will end up in wrong state.");

/* Button value fetching */
#define GET_BUTTON(GPIO, PIN) gpio_get((GPIO), (PIN))

#define LEFT_JOY_BTN_GPIO_PIN   3
#define RIGHT_JOY_BTN_GPIO_PIN  4
#define DOWN_JOY_BTN_GPIO_PIN   6
#define PRESS_JOY_BTN_GPIO_PIN  0

#define GET_BUTTON_LEFT()  (!gpio_get(GPIOB, LEFT_JOY_BTN_GPIO_PIN))
#define GET_BUTTON_RIGHT() (!gpio_get(GPIOB, RIGHT_JOY_BTN_GPIO_PIN))
#define GET_BUTTON_DOWN() (!gpio_get(GPIOB, DOWN_JOY_BTN_GPIO_PIN))
#define GET_BUTTON_PRESS() (!gpio_get(GPIOB, PRESS_JOY_BTN_GPIO_PIN))

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

    /* Enable GPIOB */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

    /* Set mode for PB5: alternative function */
    GPIOB->MODER |= GPIO_MODER_MODER5_1;

    /* Set alternative function for output pin (TIM3_CH2 for PB_5 - AF_2) */
    GPIOB->AFR[0] |= GPIO_AFRL_AFSEL5_1;

    /* Disable no PU_PD for PB_5 */
    GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD5;

    /* Disable output */
    DisableOutput();

    /* Set frequency to 44.1Khz */
    TIM3->ARR = TIM_PWM_BASE_CLOCK / 44'100;

    /* Set PWM1 function on CH2 */
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;

    /* Start timer; UG: set counter to 0 when overflow */
    TIM3->EGR |= TIM_EGR_UG;
}

//void SetupForPeriodicQuery_Tim2(void) {
//	/* Enable TIM2 clock. */
//	rcc_periph_clock_enable(RCC_TIM2);
//
//	/* Enable TIM2 interrupt. */
//	nvic_enable_irq(NVIC_TIM2_IRQ);
//
//	/* Reset TIM2 peripheral to defaults. */
//	rcc_periph_reset_pulse(RST_TIM2);
//
//	/* Timer global mode: No divider, Alignment edge, Direction up */
//	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
//
//	/* Disable preload. */
//	timer_disable_preload(TIM2);
//	timer_continuous_mode(TIM2);
//
//	/* Set timer to 10ms */
//    timer_set_period(TIM2, 500);
//	timer_set_prescaler(TIM2, (rcc_apb1_frequency * 2) / 50'000);
//
//	/* Counter enable. */
//	timer_enable_counter(TIM2);
//
//	/* Enable interrupt */
//	timer_enable_irq(TIM2, TIM_DIER_CC1IE);
//
//    /* Set maximal priority of interrupt */
//    nvic_set_priority(NVIC_TIM2_IRQ, 0);
//}

int main(void) {
    Setup96MhzClock();

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    GPIOoutConfigure(GPIOA, 6, GPIO_OType_PP, GPIO_Low_Speed, GPIO_PuPd_NOPULL);

    /*
    GPIOinConfigure(GPIOB, LEFT_JOY_BTN_GPIO_PIN,
                    GPIO_PuPd_NOPULL,
                    EXTI_Mode_Interrupt,
                    EXTI_Trigger_Rising_Falling);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, LEFT_JOY_BTN_GPIO_PIN);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, RIGHT_JOY_BTN_GPIO_PIN);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, DOWN_JOY_BTN_GPIO_PIN);
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PRESS_JOY_BTN_GPIO_PIN);
    */

    SetupForPWM_Tim3_PB5();
    // SetupForPeriodicQuery_Tim2();

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
        SET_PLAYBACK_LED_INDICATOR();
    } else {
        DisableOutput();
        NVIC_DisableIRQ(TIM3_IRQn);
        CLEAR_PLAYBACK_LED_INDICATOR();
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

/*
 * Song position control
 */

struct button_state {
    uint16_t warmup;
    uint16_t coolout;

    bool right_now_pressed;
};

bool CheckButton(struct button_state* state);

struct button_state gLeftState = {};
struct button_state gRightState = {};
struct button_state gDownState = {};
struct button_state gPressState = {};

bool CheckButton(struct button_state* state) {
    if (state->coolout) {
        state->coolout -= 1;
        return false;
    }

    if (!state->right_now_pressed) {
        state->warmup = 0;
        return false;
    }

    state->warmup += 1;

    if (state->warmup ==  WARMUP_COUNT) {
        state->warmup = 0;
        state->coolout = COOLOUT_COUNT;
        return true;
    }

    return false;
}

/* Every 10 ms */
/*
void tim2_isr(void) {
    if (!(TIM_SR(TIM2) & TIM_SR_CC1IF))
        return;

    gLeftState.right_now_pressed = GET_BUTTON_LEFT();
    gRightState.right_now_pressed = GET_BUTTON_RIGHT();
    gDownState.right_now_pressed = GET_BUTTON_DOWN();
    gPressState.right_now_pressed = GET_BUTTON_PRESS();

    if (CheckButton(&gLeftState)) {
        MoveSongBackward();
    }

    if (CheckButton(&gRightState))
        MoveSongForward();

    if (CheckButton(&gDownState))
        ResetSongPosition();

    if (CheckButton(&gPressState)) {
        gSongPlayback ^= true;
        SetSongPlayback();
    }

    TIM_SR(TIM2) &= ~TIM_SR_CC1IF;
}
*/
