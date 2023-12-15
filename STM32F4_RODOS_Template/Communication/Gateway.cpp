#include "rodos.h"

#include "Telecomand.hpp"
#include "Telemetry.hpp"
#include "Camera.hpp"
/*
// UART setup
static HAL_UART uart(UART_IDX3);
static int init_dummy = uart.init(115200);

// Gateway setup
static LinkinterfaceUART uart_linkinterface(&uart);
static Gateway uart_gateway(&uart_linkinterface);

// Init before scheduling (dont know why, taken from examples -Max)
class GatewayInitiator : public Initiator
{
    void init()
    {
        // Add Topic to forward
        uart_gateway.resetTopicsToForward();
        uart_gateway.addTopicsToForward(&telecommandTopic);
        uart_gateway.addTopicsToForward(&telemetryContinuousTopic);
        uart_gateway.addTopicsToForward(&telemetryExtendedContinuousTopic);
        uart_gateway.addTopicsToForward(&telemetryCalibIMUTopic);
        uart_gateway.addTopicsToForward(&telemetryControlParamsTopic);
        uart_gateway.addTopicsToForward(&cameraDataTopic);

    }
};

GatewayInitiator gatewayinitiator;
*/
