#include <init.h>


void SystemClock_Config(void) {
    // We will configure the main system clock to 100 MHz using the STM32's internal HSI oscillator

    FLASH->ACR |= FLASH_ACR_LATENCY_3WS; /* STM32 flash memory has certain access time, at higher clock speeds, cpu might read instructions faster
     	 	 	 	 	 	 	 	 	 	than flash can respond. For 100 Mhz, 3 wait states are required per datasheet*/

    /* RCC is the Reset and Clock Control peripheral, RCC->CR is the Reset Clock Control Register which is used for clock management.
    i.e. which oscillator to use: HSI (16 MHz High Speed Internal oscillator) or HSE (High Speed External oscillator).
    Also controls ready flags for HSI/HSE and PLL (Phase Locked Loop) */

    RCC->CR |= RCC_CR_HSION; // Enable the high speed 16 Mhz oscillator (HSI)
    while(!(RCC->CR & RCC_CR_HSIRDY)); // Wait until oscillator stabilizes: HSIRDY is status flag bit indicating HSI is stable

    RCC->PLLCFGR = (16 << RCC_PLLCFGR_PLLM_Pos) |  // PLLM = 16 (PLLM is divider so divide 16 MHz HSI clock by 16 to get 1 MHz)
                   (200 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 200 (PLLN is multiplier, i.e. 1 * 200 = 200 Mhz, PLLN must be between 192 and 432 when using USB)
                   (0 << RCC_PLLCFGR_PLLP_Pos);    // PLLP = 2 (PLLP is divider, 0 in register = divide by 2 to get 100 Mhz
    // Hence, PLL output = (HSI / PLLM) * PLLN / PLLP = (16/16)*200/2 = 100 MHz

    RCC->CR |= RCC_CR_PLLON; // Set this bit to enable PLL (Phase Locked Loop)
    while(!(RCC->CR & RCC_CR_PLLRDY)); // Wait for PLL Lock

    RCC->CFGR |= RCC_CFGR_SW_PLL; // Switches to PLL (from HSI) as system clock source
    while((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL); // Confirms that system clock source is switched to PLL
    														   // CFGR is clock configuration register
        													   // CFGR_SWS_Msk is bitmask that (SWS[1:0] bits)
        													   // CFGR_SWS_PLL is value of PLL which represents the current clock source
    														   // Loops until SWS[1:0] bits matches SWS_PLL, confirming that PLL is system clock

    RCC->CFGR |= RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1;    // Sets APB1 prescaler to /2 (50 MHz), APB2 to /1 (100 MHz)
    
    // Configure SysTick for 1ms interrupts
    SysTick_Config(100000000 / 1000);  // 100MHz / 1000 = 100,000 ticks per ms
}
