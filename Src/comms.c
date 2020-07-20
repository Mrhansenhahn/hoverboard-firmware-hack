#include <stdio.h>
#include <string.h>
#include "at32f4xx.h"
#include "defines.h"
#include "setup.h"
#include "config.h"

#ifdef DEBUG_SERIAL_USART3
#define UART_DMA_CHANNEL DMA1_Channel2
#endif

#ifdef DEBUG_SERIAL_USART2
#define UART_DMA_CHANNEL DMA1_Channel7
#endif


volatile uint8_t uart_buf[100];
volatile int16_t ch_buf[8];
//volatile char char_buf[300];

void setScopeChannel(uint8_t ch, int16_t val) {
  ch_buf[ch] = val;
}

void consoleScope() {
  #if defined DEBUG_SERIAL_SERVOTERM && (defined DEBUG_SERIAL_USART2 || defined DEBUG_SERIAL_USART3)
    uart_buf[0] = 0xff;
    uart_buf[1] = CLAMP(ch_buf[0]+127, 0, 255);
    uart_buf[2] = CLAMP(ch_buf[1]+127, 0, 255);
    uart_buf[3] = CLAMP(ch_buf[2]+127, 0, 255);
    uart_buf[4] = CLAMP(ch_buf[3]+127, 0, 255);
    uart_buf[5] = CLAMP(ch_buf[4]+127, 0, 255);
    uart_buf[6] = CLAMP(ch_buf[5]+127, 0, 255);
    uart_buf[7] = CLAMP(ch_buf[6]+127, 0, 255);
    uart_buf[8] = CLAMP(ch_buf[7]+127, 0, 255);
    uart_buf[9] = '\n';

    if(UART_DMA_CHANNEL->CNDTR == 0) {
      UART_DMA_CHANNEL->CCR &= ~DMA_CCR_EN;
      UART_DMA_CHANNEL->CNDTR = 10;
      UART_DMA_CHANNEL->CMAR  = (uint32_t)uart_buf;
      UART_DMA_CHANNEL->CCR |= DMA_CCR_EN;
    }
  #endif

  #if defined DEBUG_SERIAL_ASCII && (defined DEBUG_SERIAL_USART2 || defined DEBUG_SERIAL_USART3)
    consoleLog(get_diagnostic_info());
  #endif
}

#ifdef DEBUG_SERIAL_USART2
void consoleLog(char *message)
{
  if (UART_DMA_CHANNEL->TCNT == 0) {
    DMA_ChannelEnable(UART_DMA_CHANNEL, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_GL7 | DMA1_FLAG_TC7 | DMA1_FLAG_HT7);
    DMA_SetCurrDataCounter(UART_DMA_CHANNEL, strlen((char *)uart_buf));
    UART_DMA_CHANNEL->CMBA = (uint32_t)uart_buf;
    DMA_ChannelEnable(UART_DMA_CHANNEL, ENABLE);
  }
}
#endif // DEBUG_SERIAL_USART2

char* get_diagnostic_info() {
       memset((void*) uart_buf, 0, sizeof(uart_buf));
       sprintf((char*) uart_buf, "1:%i 2:%i 3:%i 4:%i 5:%i 6:%i 7:%i 8:%i",
                       ch_buf[0], ch_buf[1], ch_buf[2], ch_buf[3], ch_buf[4], ch_buf[5],
                       ch_buf[6], ch_buf[7]);
       return (char*) uart_buf;
}