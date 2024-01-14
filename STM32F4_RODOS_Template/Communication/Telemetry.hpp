#ifndef FLOATSAT_COMMUNICATION_TELEMETRY_HPP_
#define FLOATSAT_COMMUNICATION_TELEMETRY_HPP_

#include "rodos.h"

#include "TelemetryList.hpp"
#include "Telecomand.hpp"

#include "../InnerLoop/ReactionwheelControl.hpp"
#include "../InnerLoop/ReactionwheelEncoder.hpp"
#include "../InnerLoop/InnerLoopTopics.hpp"

#include "../OuterLoop/IMU.hpp"
#include "../OuterLoop/AttitudeEstimation.hpp"
#include "../OuterLoop/AngularPositionControl.hpp"
#include "../OuterLoop/AngularVelocityControl.hpp"

#include "../Docking/DockingTopics.hpp"

#include "../Electrical/ElectricalMonitoring.hpp"

#include "../Modes.hpp"
#include "../PIDController.hpp"


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
