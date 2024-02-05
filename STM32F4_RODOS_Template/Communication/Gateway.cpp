#include "rodos.h"

#include "Telecomand.hpp"
#include "Telemetry.hpp"
#include "Camera.hpp"

// UART setup
static HAL_UART uart(UART_IDX2, GPIO_053, GPIO_054);
//static int init_dummy = uart.init(115200);

// Gateway setup
static LinkinterfaceUART uart_linkinterface(&uart, 115200);
static Gateway uart_gateway(&uart_linkinterface);

// Init before scheduling (dont know why, taken from examples -Max)
class GatewayInitiator : public Initiator
{
    void init()
    {   

        //uart_linkinterface.init();
        // Add Topic to forward
        uart_gateway.resetTopicsToForward();
        uart_gateway.addTopicsToForward(&telecommandTopic);
        uart_gateway.addTopicsToForward(&telemetryContinuousTopic);
        uart_gateway.addTopicsToForward(&telemetryExtendedContinuousTopic);
        uart_gateway.addTopicsToForward(&telemetryCalibIMUTopic);
        uart_gateway.addTopicsToForward(&telemetryControlParamsTopic);

        uart_gateway.addTopicsToForward(&cameraDataTopic);
        uart_gateway.addTopicsToForward(&cameraPwrCmdTopic);
        uart_gateway.addTopicsToForward(&cameraShutdownTopic);

        uart_gateway.addTopicsToForward(&cameraTest);

        uart_gateway.addTopicsToForward(&EchoTopic);

    }
};

GatewayInitiator gatewayinitiator;
