#ifndef FLOATSAT_COMMUNICATION_GATEWAY_HPP_
#define FLOATSAT_COMMUNICATION_GATEWAY_HPP_

#include "rodos.h"


// Set up uart gatway
static HAL_UART uart(UART_IDX3);
static int idummy = uart.init(115200);
static LinkinterfaceUART uart_linkerinterface(&uart);
static Gateway uart_gateway(&uart_linkerinterface);


#endif
