#include "configuration.h"
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

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

void SetupForPWM_Tim3_PC7(void) {
    /* Enable peripherial clock for GPIOC. */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    /* Enable TIM3 */
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;

    /* Disable prescaller (only useful when timing long events) */
    TIM3->PSC = 0;

    /* Enable ARPE and enable counter itself */
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

    /* Disable output on CH2 */
    TIM3->CCER &= ~0x0010;
    NVIC_DisableIRQ(TIM3_IRQn);

    /* Set frequency to 44.1Khz */
    TIM3->ARR = BASE_CLOCK / 44'100;

    /* Set PWM1 function on CH2 */
    TIM3->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2PE;

    /* Start timer; UG: set counter to 0 when overflow */
    TIM3->EGR |= TIM_EGR_UG;
}

void SetupForPeriodicQuery_Tim2(void) {
    /* Enable TIM2 clock. */
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    /* Set timer to 10ms */
    TIM2->ARR = 500;
    TIM2->PSC = BASE_CLOCK / 50'000;

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

void SetupLedButtonsInterrupts(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN |
                    RCC_AHB1ENR_GPIOBEN;

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
}
