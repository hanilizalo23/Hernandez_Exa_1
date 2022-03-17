/**
	\file
	\brief P1, Examen 1
	\author Nelida Paulina Hernández Moya
	\date	17/03/2022
 */

#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

#include "fsl_common.h"
#include "PIT.h"
#include "Bits.h"
#include "GPIO.h"
#include "NVIC.h"
#include "RGB.h"

#define SYSTEM_CLOCK 	    (21000000U)
#define DELAY_WHITE   		(2U)
#define DELAY_FOR_GREEN  	(4U)
#define DELAY_SW3			(10U)
#define DELAY_COLORS		(1U)
#define NOTHING				(0U)

typedef enum {INIT, WHITE, GREEN, BLUE, RED, YELLOW, PURPLE, CIAN} main_states_t;

void init_routine (void);
void change_state (void);
void white_sequence (void);
void sw3_counter (void);
void toggling_colors (void);

static main_states_t state_main = INIT;
static uint8_t g_white = FALSE;
static uint8_t g_turning = 0;
static uint8_t g_sw3_times = 0;

int main(void)
{
	gpio_pin_control_register_t pcr_gpioa_pin_4 = GPIO_MUX1 | GPIO_PE | GPIO_PS | INTR_FALLING_EDGE;
	GPIO_clock_gating(GPIO_A);
	GPIO_pin_control_register(GPIO_A,bit_4, &pcr_gpioa_pin_4);
	GPIO_data_direction_pin(GPIO_A, GPIO_INPUT, bit_4);

	gpio_pin_control_register_t pcr_gpioc_pin_6 = GPIO_MUX1 | GPIO_PE | GPIO_PS | INTR_FALLING_EDGE;
	GPIO_clock_gating(GPIO_C);
	GPIO_pin_control_register(GPIO_C,bit_6,  &pcr_gpioc_pin_6);
	GPIO_data_direction_pin(GPIO_C, GPIO_INPUT, bit_6);

	NVIC_set_basepri_threshold(PRIORITY_10);
	NVIC_enable_interrupt_and_priotity(PORTC_IRQ,PRIORITY_2);
	NVIC_enable_interrupt_and_priotity(PORTA_IRQ,PRIORITY_1);
	NVIC_global_enable_interrupts;

	PIT_clock_gating();
	PIT_enable();

	PIT_enable_interrupt(PIT_0);
	NVIC_enable_interrupt_and_priotity(PIT_CH0_IRQ, PRIORITY_3);
	PIT_callback_init(PIT_0, white_sequence);

	PIT_enable_interrupt(PIT_1);
	NVIC_enable_interrupt_and_priotity(PIT_CH1_IRQ, PRIORITY_4);
	PIT_callback_init(PIT_1, change_state);

	PIT_enable_interrupt(PIT_2);
	NVIC_enable_interrupt_and_priotity(PIT_CH2_IRQ, PRIORITY_5);
	PIT_callback_init(PIT_2, toggling_colors);

    while (1)
    {
    }
