#include <stddef.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#define SET_PLAYBACK_LED_INDICATOR() gpio_set(GPIOA, GPIO6)
#define CLEAR_PLAYBACK_LED_INDICATOR gpio_clear(GPIOA, GPIO6)

/* Debouncing parameters */
#define WARMUP_COUNT 4
#define COOLOUT_COUNT 50

/* Playback adjustment step (must be divisible by 11) */
#define PLAYBACK_STEP 7568 * 10

/* Button value fetching */
#define GET_BUTTON(GPIO, PIN) gpio_get((GPIO), (PIN))

#define GET_BUTTON_LEFT()  (!gpio_get(GPIOB, GPIO3))
#define GET_BUTTON_RIGHT() (!gpio_get(GPIOB, GPIO4))
#define GET_BUTTON_UP()    (!gpio_get(GPIOB, GPIO6))
#define GET_BUTTON_PRESS() (!gpio_get(GPIOB, GPIO10))

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

const struct rcc_clock_scale rcc_clock_96MHz = {
    .pllm = 8,
    .plln = 96,
    .pllp = 2,
    .pllq = 4,
    .pllr = 0,
    .pll_source = RCC_CFGR_PLLSRC_HSI_CLK,
    .hpre = RCC_CFGR_HPRE_DIV_NONE,
    .ppre1 = RCC_CFGR_PPRE_DIV_2,
    .ppre2 = RCC_CFGR_PPRE_DIV_NONE,
    .voltage_scale = PWR_SCALE1,
    .flash_config = FLASH_ACR_DCEN | FLASH_ACR_ICEN | FLASH_ACR_LATENCY_3WS,

    /* Those parameters yield following frequency: */
    .ahb_frequency  = 96'000'000,
    .apb1_frequency = 48'000'000,
    .apb2_frequency = 96'000'000
};

void SetupForPWM_Tim3_PB5(void) {
    /* Enable TIM3 */
	rcc_periph_clock_enable(RCC_TIM3);

    /* Disable prescaller (only useful when timing long events) */
	timer_set_prescaler(TIM3, 0);

    /* When ARPE is enabled, ARR value is buffered in the Shadow Register, so
     * changing the ARR value will not have any effect on the current cycle
     * until the UEV happens */
    timer_enable_preload(TIM3);

    /* Configuration completed, enable counter */
    timer_enable_counter(TIM3);

    /* Setup interrupt */
    timer_enable_irq(TIM3, TIM_DIER_UIE);
	nvic_disable_irq(NVIC_TIM3_IRQ);

    /* Set maximal priority of interrupt */
    nvic_set_priority(NVIC_TIM3_IRQ, 0);

    /* Enable GPIOB */
    rcc_periph_clock_enable(RCC_GPIOB);

    /* Set mode for output pin */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO5);

    /* Set alternative function for output pin (TIM3_CH2 for PB_5 - AF_2) */
    gpio_set_af(GPIOB, GPIO_AF2, GPIO5);

    /* Disable output */
    DisableOutput();

    /* Set frequency to 44.1Khz */
    timer_set_period(TIM3, (rcc_apb1_frequency * 2) / 44'100);

    /* Set PWM1 function on CH2 */
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);

    /* Start timer; set counter to 0 when overflow */
	timer_enable_update_event(TIM3);
    timer_generate_event(TIM3, TIM_EGR_UG);
}

void SetupForPeriodicQuery_Tim2(void) {
	/* Enable TIM2 clock. */
	rcc_periph_clock_enable(RCC_TIM2);

	/* Enable TIM2 interrupt. */
	nvic_enable_irq(NVIC_TIM2_IRQ);

	/* Reset TIM2 peripheral to defaults. */
	rcc_periph_reset_pulse(RST_TIM2);

	/* Timer global mode: No divider, Alignment edge, Direction up */
	timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT, TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);

	/* Disable preload. */
	timer_disable_preload(TIM2);
	timer_continuous_mode(TIM2);

	/* Set timer to 10ms */
    timer_set_period(TIM2, 500);
	timer_set_prescaler(TIM2, (rcc_apb1_frequency * 2) / 50'000);

	/* Counter enable. */
	timer_enable_counter(TIM2);

	/* Enable interrupt */
	timer_enable_irq(TIM2, TIM_DIER_CC1IE);

    /* Set maximal priority of interrupt */
    nvic_set_priority(NVIC_TIM2_IRQ, 0);
}

int main(void) {
    /* Setup custom clock value */
    rcc_clock_setup_pll(&rcc_clock_96MHz);

    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);   // led
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO3);  // joystick left
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO4);  // joystick right
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO6);  // joystick down
    gpio_mode_setup(GPIOB, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO10); // joystick press

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
    TIM_CCER(TIM3) &= ~0x0010;
}

void EnableOutput(void) {
    /* Enable output on CH2 */
    TIM_CCER(TIM3) |= 0x0010;
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
    gSongIndexByte += PLAYBACK_STEP;
}

void ResetSongPosition(void) {
    gSongIndexByte = 0;
    gSongIndexBitFromMsb = 0;
}

void SetSongPlayback(void) {
    if (gSongPlayback) {
        EnableOutput();
        nvic_enable_irq(NVIC_TIM3_IRQ);
        SET_PLAYBACK_LED_INDICATOR();
    } else {
        DisableOutput();
        nvic_disable_irq(NVIC_TIM3_IRQ);
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
void tim3_isr(void) {
    if (!(TIM_SR(TIM3) & TIM_SR_UIF) || !gSongPlayback)
        return;

    uint16_t next_sample;

    if (!FetchNextSongSample(&next_sample)) {
        gSongPlayback = false;
        SetSongPlayback();
        return;
    }

    timer_set_oc_value(TIM3, TIM_OC2, next_sample);

    TIM_SR(TIM3) &= ~TIM_SR_UIF;
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
struct button_state gUpState = {};
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
void tim2_isr(void) {
    if (!(TIM_SR(TIM2) & TIM_SR_CC1IF))
        return;

    gLeftState.right_now_pressed = GET_BUTTON_LEFT();
    gRightState.right_now_pressed = GET_BUTTON_RIGHT();
    gUpState.right_now_pressed = GET_BUTTON_UP();
    gPressState.right_now_pressed = GET_BUTTON_PRESS();

    if (CheckButton(&gLeftState)) {
        MoveSongBackward();
    }

    if (CheckButton(&gRightState))
        MoveSongForward();

    if (CheckButton(&gUpState))
        ResetSongPosition();

    if (CheckButton(&gPressState)) {
        gSongPlayback ^= true;
        SetSongPlayback();
    }

    TIM_SR(TIM2) &= ~TIM_SR_CC1IF;
}
