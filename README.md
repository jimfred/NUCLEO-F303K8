# NUCLEO-F303K8
Collection of code for Nucleo-F303k8 board

## Blink_cpp_LL 
* Simple project that blinks the Nucleo's LED based on the push button. 
* It's a CPP project (CubeMX generates .C code by default) and used LL rather than HAL. I'm finding that LL (Low Level) is simpler.

## ctr_isr_cpp_LL
* Uses TIM2 to count a push-button connected between PA0 (called TIM2_ETR) and 3V3.
*  PA0 is configured with a pull down.
*  Green LED blinks but the blink rate toggles upon each TIM2 interrupt.
*  TIM2 interrupt is called after 5 button presses.
*  To debounce the button, LL_TIM_ETR_FILTER_FDIV32_N8 is used.
*  No HAL - all LL (Low Level).

## uart_ll_polled
* echos UART2 to the ST-Link USB Serial port. 
* Uses polled mode with about 3 low-level APIs.
