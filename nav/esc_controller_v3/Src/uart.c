#include <uart.h>
#include <ctype.h>

#define RX_BUFFER_SIZE 256

// Circular buffers for received data, stores values between 0-255 so 8 bit is enough
static volatile uint8_t usart1_rxBuffer[RX_BUFFER_SIZE];
static volatile uint8_t usart2_rxBuffer[RX_BUFFER_SIZE];
static volatile uint8_t usart3_rxBuffer[RX_BUFFER_SIZE];

static volatile uint16_t usart1_rxReadIndex = 0; // larger in case we need a bigger buffer in the future
static volatile uint16_t usart1_rxWriteIndex = 0;
static volatile uint16_t usart2_rxReadIndex = 0;
static volatile uint16_t usart2_rxWriteIndex = 0;
static volatile uint16_t usart3_rxReadIndex = 0;
static volatile uint16_t usart3_rxWriteIndex = 0;

/**
 * Initialize USART1 on PA9 (TX) and PA10 (RX) for Left Motor VESC
 * Initialize USART2 on PA2 (TX) and PA3 (RX) for ROS2 Bridge
 * Initialize USART3 on PC10 (TX) and PC11 (RX) for Right Motor VESC
**/

void UART1_Init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable GPIOA clock
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // Enable USART1 clock

    // Configure PA9 and PA10 mode for alternate function
    GPIOA->MODER &= ~(GPIO_MODER_MODER9 | GPIO_MODER_MODER10);
    GPIOA->MODER |= (2 << GPIO_MODER_MODER9_Pos) |   // AF mode for PA9
                    (2 << GPIO_MODER_MODER10_Pos);   // AF mode for PA10

    // Set alternate function to AF7 (USART1)
    GPIOA->AFR[1] &= ~(0xFF << 4);    // Clear AF bits for PA9 and PA10
    GPIOA->AFR[1] |= (7 << 4) |       // PA9 = AF7 (USART1_TX)
                     (7 << 8);        // PA10 = AF7 (USART1_RX)

    // Configure USART1 baud rate and enable (APB2)
    USART1->BRR = 0x364;  // Baud rate divisor -> 100 MHz / (16 * 115200) = 54.25 → 0x364
    USART1->CR1 = USART_CR1_TE |    // Transmitter enable
                  USART_CR1_RE |    // Receiver enable
                  USART_CR1_RXNEIE |
                  USART_CR1_UE;     // USART enable
    
    NVIC_SetPriority(USART1_IRQn, 2); // Set priority (0 is highest)
    NVIC_EnableIRQ(USART1_IRQn);      // Enable the interrupt
}

void UART2_Init(void) {
    // Enable peripheral clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // Enable GPIOA clock (which enables PA2 and PA3 which is used by USART2)
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable USART2 clock

    // Configure GPIO pins
    GPIOA->MODER &= ~(GPIO_MODER_MODER2 | GPIO_MODER_MODER3); // Clear mode bits associated with PA2 and PA3
    GPIOA->MODER |= (2 << GPIO_MODER_MODER2_Pos) |  // Set PA2 to alternate function mode
                    (2 << GPIO_MODER_MODER3_Pos);    // Set PA3 to alternate function mode

    // Configure alternate function of PA2 (TX) and PA3 (RX) to AF7 which maps to USART2
    GPIOA->AFR[0] &= ~(0xFF << 8); // Clear AF bits for PA2 and PA3
    GPIOA->AFR[0] |= (7 << 8) |    // Set PA2 to AF7 (USART2_TX)
                     (7 << 12);    // Set PA3 to AF7 (USART2_RX)

    // Configure USART2 baud rate and enable
    USART2->BRR = 0x1B2; // 50 MHz / (16 * 115200) ≈ 27.1267 → 0x1B2
    USART2->CR1 = USART_CR1_TE |    // Transmitter enable
                  USART_CR1_RE |    // Receiver enable
                  USART_CR1_RXNEIE | // RX interrupt enable
                  USART_CR1_UE;     // USART enable
    
    // Enable USART2 interrupt in NVIC
    NVIC_SetPriority(USART2_IRQn, 1); // Set priority (0 is highest)
    NVIC_EnableIRQ(USART2_IRQn);      // Enable the interrupt
}

void UART3_Init(void) {
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;   // Enable GPIOC clock
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;  // Enable USART3 clock

    // Configure PC10 (TX) and PC11 (RX) as alternate function
    GPIOC->MODER &= ~(GPIO_MODER_MODER10 | GPIO_MODER_MODER11);
    GPIOC->MODER |= (2 << GPIO_MODER_MODER10_Pos) |  // AF mode for PC10
                    (2 << GPIO_MODER_MODER11_Pos);   // AF mode for PC11

    // Set alternate function to AF7 (USART3)
    GPIOC->AFR[1] &= ~(0xFF << 8);   // Clear AF bits for PC10 and PC11
    GPIOC->AFR[1] |= (7 << 8) |      // PC10 = AF7 (USART3_TX)
                     (7 << 12);      // PC11 = AF7 (USART3_RX)

    // Configure USART3 baud rate and enable
    USART3->BRR = 0x1B2;  // 50 MHz / (16 * 115200) ≈ 27.1267 → 0x1B2
    USART3->CR1 = USART_CR1_TE |    // Transmitter enable
                  USART_CR1_RE |    // Receiver enable
                  USART_CR1_RXNEIE |
                  USART_CR1_UE;     // USART enable

    NVIC_SetPriority(USART3_IRQn, 2); // Set priority (0 is highest)
    NVIC_EnableIRQ(USART3_IRQn);      // Enable the interrupt
}

// Initialize all 3 UART ports
void UART_Init(void) {
    UART1_Init();  // Left motor VESC
    UART2_Init();  // ROS2 bridge
    UART3_Init();  // Right motor VESC
}


// Send single byte over specified USART (specified by *usart)
void UART_SendByte(USART_TypeDef *usart, uint8_t byte) {
    while(!(usart->SR & USART_SR_TXE));
    usart->DR = byte;
}

// Send string over UART to ROS2 Control Computer
void UART_SendString(uint8_t uart_id, const char *str) {
    USART_TypeDef *usart;
    
    switch(uart_id) {
        case 1: usart = USART1; break;
        case 2: usart = USART2; break;
        case 3: usart = USART3; break;
        default: return;  // Invalid UART ID
    }
    
    while (*str) {
        UART_SendByte(usart, (uint8_t)*str++);
    }
}

uint8_t UART_DataAvailable(uint8_t uart_id) {
    switch(uart_id) {
        case 1:
            return (usart1_rxReadIndex != usart1_rxWriteIndex);
            
        case 2:
            return (usart2_rxReadIndex != usart2_rxWriteIndex);
            
        case 3:
            return (usart3_rxReadIndex != usart3_rxWriteIndex);
            
        default:
            return 0;  // Invalid UART ID - no data
    }
}


// Common RX interrupt handler logic
static void USART_RX_Handler(USART_TypeDef *usart, 
                             volatile uint8_t *buffer, 
                             volatile uint16_t *writeIndex, 
                             volatile uint16_t *readIndex) {
    if (usart->SR & USART_SR_RXNE) {
        // Read received data
        uint8_t rxData = usart->DR;
        
        // Calculate next write index
        uint16_t nextWriteIndex = (*writeIndex + 1) % RX_BUFFER_SIZE;
        
        // If buffer is not full, store the data
        if (nextWriteIndex != *readIndex) {
            buffer[*writeIndex] = rxData;
            *writeIndex = nextWriteIndex;
        }
    }
}

// USART1 Interrupt Handler
void USART1_IRQHandler(void) {
    USART_RX_Handler(USART1, usart1_rxBuffer, &usart1_rxWriteIndex, &usart1_rxReadIndex);
}

// USART2 Interrupt Handler
void USART2_IRQHandler(void) {
    USART_RX_Handler(USART2, usart2_rxBuffer, &usart2_rxWriteIndex, &usart2_rxReadIndex);
}

// USART3 Interrupt Handler
void USART3_IRQHandler(void) {
    USART_RX_Handler(USART3, usart3_rxBuffer, &usart3_rxWriteIndex, &usart3_rxReadIndex);
}

uint8_t UART_ReadByte(uint8_t uart_id) {
    uint8_t byte;
    switch(uart_id) {
        case 1:
            byte = usart1_rxBuffer[usart1_rxReadIndex];
            usart1_rxReadIndex = (usart1_rxReadIndex + 1) % RX_BUFFER_SIZE;
            break;
        case 2:
            byte = usart2_rxBuffer[usart2_rxReadIndex];
            usart2_rxReadIndex = (usart2_rxReadIndex + 1) % RX_BUFFER_SIZE;
            break;
        case 3:
            byte = usart3_rxBuffer[usart3_rxReadIndex];
            usart3_rxReadIndex = (usart3_rxReadIndex + 1) % RX_BUFFER_SIZE;
            break;
        default:
            byte = 0;
            break;
    }
    return byte;
}