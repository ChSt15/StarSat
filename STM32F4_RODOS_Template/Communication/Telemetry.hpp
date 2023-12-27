#ifndef FLOATSAT_COMMUNICATION_TELEMETRY_HPP_
#define FLOATSAT_COMMUNICATION_TELEMETRY_HPP_

#include "rodos.h"

#include "TelemetryList.hpp"
#include "../hardware/imu.hpp"
#include "../control/AttitudeEstimation.hpp"
#include "../control/ReactionwheelControl.hpp"
#include "../control/AngularPositionControl.hpp"
#include "../control/AngularVelocityControl.hpp"
#include "../control/PIDController.hpp"
#include "../hardware/ReactionwheelEncoder.hpp"
#include "../Threads/Modes.hpp"
#include "Telecomand.hpp"


class Telemetry
{
private:

	TelemetryContinuous telemetry_continuous;
	TelemetryCalibIMU telemetry_calib;
	TelemetryControlParams telemetry_control;
	TelemetryContinuousExtended telemetry_extended;

	bool enable_extendedtelem = false;

public:

	// @brief Sends telemetry 
	void send_Continuous();

	// @brief Sends IMU calib telemetry
	void send_CalibIMU();

	// @brief Sends control params telemetry
	void send_ControlParams();

	void enable_ExtendedTelemetry(bool enable);
};

// @brief Global telemetry object
extern Telemetry telemetry;

// @brief Global countinuos telemetry topic
extern Topic<TelemetryContinuous> telemetryContinuousTopic;

// @brief Global extended countinuos telemetry topic
extern Topic<TelemetryContinuousExtended> telemetryExtendedContinuousTopic;

// @brief Global IMU calib telemetry topic
extern Topic<TelemetryCalibIMU> telemetryCalibIMUTopic;

// @brief Global control params telemetry topic
extern Topic<TelemetryControlParams> telemetryControlParamsTopic;


#endif
