#ifndef FLOATSAT_COMMUNICATION_TELECOMANDLIST_HPP_
#define FLOATSAT_COMMUNICATION_TELECOMANDLIST_HPP_

#define TelecommandTopicId 50

// @brief Structure of a Telecommand
struct Command
{
	int id;
	float fval_1, fval_2, fval_3;
};

// @brief List of all telecommandID
enum CommandIds
{
	/*-----------------------------Modes----------------------------*/
	// Changes mode to Idle
	SetMode_Idle = 1,
	// Changes mode to Calib_Gyro
	SetMode_CalibGyro,	
	// Changes mode to Calib_Accel
	SetMode_CalibAccel,	
	// Changes mode to Calib_Mag
	SetMode_CalibMag,
	// Changes mode to Calib_Arm
	SetMode_CalibArm,	
	// Changes mode to Control_Speed
	SetMode_ControlSpeed ,	
	// Changes mode to Control_Pos
	SetMode_ControlPos,	
	// Changes mode to Control_Vel
	SetMode_ControlVel,	
	// Changes mode to Mission (Beginn)
	SetMode_Mission,	
	/*--------------------------Calib Parms-------------------------*/
	// Set offset for gyro (x, y, z)
	SetCalibParams_gyro,
	// Set offset for accel (x, y, z)
	SetCalibParams_accel,	
	// Set offset for mag (x, y, z)
	SetCalibParams_mag,	
	/*-------------------------Control Parms------------------------*/
	// Set PID params for speed controler (P, I, D)
	SetControlParams_speed,
	// Set limit for speed controler
	SetControlLimit_speed,
	// Set PID params for pos controler (P, I, D)
	SetControlParams_pos,	
	// Set limit for pos controler
	SetControlLimit_pos,
	// Set PID params for vel controler (P, I, D)
	SetControlParams_vel,	
	// Set limit for vel controler
	SetControlLimit_vel,	
	/*-----------------------Control Setpoint----------------------*/
	// Set desired value for speed control
	SetControlDesired_speed,
	// Set desired value for pos control
	SetControlDesired_pos,	
	// Set desired value for vel control
	SetControlDesired_vel,	
	/*----------------------------Telemtry--------------------------*/
	// Send IMU Calib Telemetry once 
	SendCalibTelemetry, 
	// Send Control Param Telemetry once 
	SendControlTelemetry,
	// Toggles extended Telemetry (fval_1 < 0 -> off, fval_1 > 0 -> on)
	ToggleExtendedTelemetry,
	/*-----------------------------Camera---------------------------*/
	// Toggles Camera Enable (fval_1 < 0 -> off, fval_1 > 0 -> on)
	ToggleCamera,
	// Toggles Video Stream (fval_1 < 0 -> off, fval_1 > 0 -> on)
	ToggleVideoStreaming

};

#endif
