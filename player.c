#include <stddef.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

void DisableOutput(int channel);
void EnableOutput(int channel);
void SetupSongPlayback(const unsigned char* song_data, size_t song_data_size);
bool FetchNextSongSample(uint16_t* write_to);

const unsigned char sound1[] = {
// #embed "sound_funny.raw11" soon
#include "sound_funny.raw11.cdata"
};

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

void DisableOutput(int channel) {
    /* Disable output on CH2 */
    TIM_CCER(TIM3) &= ~0x0010;
}

void EnableOutput(int channel) {
    /* Enable output on CH2 */
    TIM_CCER(TIM3) |= 0x0010;
}

int main(void) {
    /* Setup custom clock value */
    rcc_clock_setup_pll(&rcc_clock_96MHz);

    rcc_periph_clock_enable(RCC_GPIOA);
    gpio_mode_setup(GPIOA, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO5);

    gpio_toggle(GPIOA, GPIO5);


    /* Enable TIM3 */
	rcc_periph_clock_enable(RCC_TIM3);

    /* Disable prescaller (It is only useful when timing long events) */
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
    timer_set_period(TIM3, rcc_ahb_frequency / 44'100);

    /* Set PWM1 function on CH2 */
    timer_set_oc_mode(TIM3, TIM_OC2, TIM_OCM_PWM1);

    /* Start timer; set counter to 0 when overflow */
	timer_enable_update_event(TIM3);
    timer_generate_event(TIM3, TIM_EGR_UG);

    /* Play song */
    SetupSongPlayback(sound1, sizeof(sound1) / sizeof(*sound1));

    while (true);
}

const unsigned char* gSongData = NULL;

static uint32_t gSongDataSize = 0;
static uint32_t gSongIndexByte = 0;
static uint32_t gSongIndexBitFromMsb = 0;

void SetupSongPlayback(const unsigned char* song_data, size_t song_data_size) {
    gSongData = song_data;
    gSongDataSize = song_data_size;
    gSongIndexByte = 0;
    gSongIndexBitFromMsb = 0;

    EnableOutput();
    nvic_enable_irq(NVIC_TIM3_IRQ);
}

/* Unpacking bit-packed data is harder than solving L*etCode lol */
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

void tim3_isr(void) {
    gpio_toggle(GPIOA, GPIO5);

    if (!(TIM_SR(TIM3) & TIM_SR_UIF))
        return;

    uint16_t next_sample;

    if (!FetchNextSongSample(&next_sample)) {
	    nvic_disable_irq(NVIC_TIM3_IRQ);
        DisableOutput();
    }

    timer_set_oc_value(TIM3, TIM_OC2, next_sample);

    TIM_SR(TIM3) &= ~TIM_SR_UIF;
}
