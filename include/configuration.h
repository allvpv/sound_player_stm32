#pragma once
#include <stm32.h>
#include <gpio.h>

#define BASE_CLOCK 96'000'000

void Setup96MhzClock(void);
void SetupForPWM_Tim3_PC7(void);
void SetupForPeriodicQuery_Tim2(void);
void SetupLedButtonsInterrupts(void);

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

#define RED_LED_GPIO    GPIOA
#define GREEN_LED_GPIO  GPIOA

#define RED_LED_PIN     6
#define GREEN_LED_PIN   7

#define LEFT_JOY_BTN_PIN   3
#define RIGHT_JOY_BTN_PIN  4
#define DOWN_JOY_BTN_PIN   6
#define PRESS_JOY_BTN_PIN  10

#define IS_BUTTON_ON(GPIO, PIN) (!(((GPIO)->IDR >> (PIN)) & 1))

#define GET_BUTTON_LEFT()  IS_BUTTON_ON(GPIOB, LEFT_JOY_BTN_PIN)
#define GET_BUTTON_RIGHT() IS_BUTTON_ON(GPIOB, RIGHT_JOY_BTN_PIN)
#define GET_BUTTON_DOWN()  IS_BUTTON_ON(GPIOB, DOWN_JOY_BTN_PIN)
#define GET_BUTTON_PRESS() IS_BUTTON_ON(GPIOB, PRESS_JOY_BTN_PIN)

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
