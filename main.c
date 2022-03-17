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

typedef enum {INIT, WHITE, GREEN, BLUE, RED, YELLOW, PURPLE, CYAN} main_states_t;

void init_routine (void);
void change_state (void);
void white_sequence (void);
void sw3_counter (void);
void toggling_colors (void);

static main_states_t state_main = INIT;
static uint8_t g_white = FALSE;
static uint8_t g_turning = 0;
static uint8_t g_sw3_times = 0;

uint32_t (*p_color[6]) (int color) = {blue_on, green_on, red_on, yellow_on, purple_on, cyan_on};

int main(void)
{
	/**Configuraciones*/
	gpio_pin_control_register_t pcr_gpioa_pin_4 = GPIO_MUX1 | GPIO_PE | GPIO_PS | INTR_FALLING_EDGE;
	GPIO_clock_gating(GPIO_A);
	GPIO_pin_control_register(GPIO_A,bit_4, &pcr_gpioa_pin_4);
	GPIO_data_direction_pin(GPIO_A, GPIO_INPUT, bit_4);

	gpio_pin_control_register_t pcr_gpioc_pin_6 = GPIO_MUX1 | GPIO_PE | GPIO_PS | INTR_FALLING_EDGE;
	GPIO_clock_gating(GPIO_C);
	GPIO_pin_control_register(GPIO_C,bit_6,  &pcr_gpioc_pin_6);
	GPIO_data_direction_pin(GPIO_C, GPIO_INPUT, bit_6);

	GPIO_callback_init(GPIO_C, init_routine);
	GPIO_callback_init(GPIO_A, sw3_counter);

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
    	/**Máquina de estados*/
    	switch(state_main)
    	{
    	case INIT:
    		rgb_init();
    		state_main = GREEN;
    		break;

    	case GREEN:
    		if(DELAY_FOR_GREEN <= g_turning)
    		{
    			PIT_stop(PIT_0);
    			(*p_color[1]) (green_on);
    			g_turning = NOTHING;
    			PIT_delay(PIT_1, SYSTEM_CLOCK, DELAY_SW3);
    		}
    		break;

    	case BLUE:
    		rgb_off();
    		(*p_color[0]) (blue_on);
    	break;

    	case RED:
    		rgb_off();
    		(*p_color[2]) (red_on);
    	break;

    	case PURPLE:
    		rgb_off();
    		(*p_color[4]) (purple_on);
    	break;

    	case YELLOW:
    		rgb_off();
    		(*p_color[3]) (yellow_on);
    	break;

    	case CYAN:
    		rgb_off();
    		(*p_color[5]) (cyan_on);
    	break;

    	default:
    		break;
    	}
    }
}

/**Secuencia de arranque*/
void init_routine (void)
{
	white_on();
	PIT_delay(PIT_0, SYSTEM_CLOCK, DELAY_WHITE);
	g_white = TRUE;
	return;
}

/**Secuencia de blanco*/
void white_sequence (void)
{
	if(DELAY_WHITE >= g_turning)
	{
		if(TRUE == g_white)
		{
			g_white = FALSE;
			rgb_off();
		}
		else
		{
			g_white = TRUE;
			white_on();
		}
	}
	g_turning++;
}

/**Cambiamos de estado conforme el anterior*/
void change_state (void)
{
	g_sw3_times++;
	switch(g_sw3_times)
	{
		case GREEN:
			state_main = GREEN;
		break;

		case BLUE:
			state_main = BLUE;
		break;

		case RED:
			state_main = RED;
		break;

		case PURPLE:
			state_main = PURPLE;
		break;

		case CYAN:
			state_main = CYAN;
		break;

		default:
			break;
	}
	g_sw3_times = NOTHING;
	PIT_delay(PIT_2, SYSTEM_CLOCK, DELAY_COLORS);
	return;
}

/**Contador para los 5 segundos*/
void sw3_counter (void)
{
	if(!GPIO_read_pin(GPIO_A, bit_4))
	{
		g_sw3_times++;
	}
}

/**Cambiamos los colores cada 500 mS*/
void toggling_colors (void)
{
	if(GREEN <= state_main)
	{

	}
}
