#include "Telecomand.hpp"


// Telecomand topic
Topic<Command> telecommandTopic(TelecommandTopicId, "Telecomand Topic");

// Telecomand echo  topic
Topic<Command> EchoTopic(TelecommandEchoTopicId, "Echo Topic");

// Telecomand topic subscriber setup
static Fifo<Command, 10> commandFIFO;
static Subscriber telecommandSubsciber(telecommandTopic, commandFIFO, "Telecommand Class");
Command commandReceiver;

void Telecommand::processNewCommand()
{
	PIDParams params;
	IMUCalib calib;

	// Work through FIFO queue
	while (commandFIFO.get(commandReceiver))
	{
		EchoTopic.publish(commandReceiver);
		switch ((CommandIds) commandReceiver.id)
		{
		/*-----------------------------Modes----------------------------*/
		case SetMode_Idle:
			setMode(Idle);
			break;
		case SetMode_CalibGyro:
			setMode(Calib_Gyro);
			break;
		case SetMode_CalibAccel:
			setMode(Calib_Accel);
			break;
		case SetMode_CalibMag:
			setMode(Calib_Mag);
			break;
		case SetMode_CalibArm:
			setMode(Calib_Arm);
			break;
		case SetMode_ControlSpeed:
			setMode(Control_Speed);
			break;
		case SetMode_ControlPos:
			setMode(Control_Pos);
			break;
		case SetMode_ControlVel:
			setMode(Control_Vel);
			break;
		case SetMode_Mission:
			setMode(Mission_Locate);
			break;
			/*
		/*--------------------------Calib Parms-------------------------*/
		case SetCalibParams_gyro:
			calib.bias.x = commandReceiver.fval_1; 
			calib.bias.y = commandReceiver.fval_2;
			calib.bias.z = commandReceiver.fval_3;
			calib.scale = Matrix3D_F();
			imu.setGyroCalib(calib);
			break;
		case SetCalibParams_accel:
			calib.bias.x = commandReceiver.fval_1;
			calib.bias.y = commandReceiver.fval_2;
			calib.bias.z = commandReceiver.fval_3;
			calib.scale = Matrix3D_F();
			imu.setGyroCalib(calib);
			break;
		case SetCalibParams_mag:
			calib.bias.x = commandReceiver.fval_1;
			calib.bias.y = commandReceiver.fval_2;
			calib.bias.z = commandReceiver.fval_3;
			calib.scale = Matrix3D_F();
			imu.setGyroCalib(calib);
			break;
		/*-------------------------Control Parms------------------------*/
		case SetControlParams_speed:
			params.kp = commandReceiver.fval_1;
			params.ki = commandReceiver.fval_2;
			params.kd = commandReceiver.fval_3;
			reactionwheelControl.setParams(params);
		case SetControlLimit_speed:
			// TODO
		case SetControlParams_pos:
			params.kp = commandReceiver.fval_1;
			params.ki = commandReceiver.fval_2;
			params.kd = commandReceiver.fval_3;
			positionControl.setParams(params);
		case SetControlLimit_pos:
			positionControl.setMaxAngularVelocity(commandReceiver.fval_1);
		case SetControlParams_vel:
			params.kp = commandReceiver.fval_1;
			params.ki = commandReceiver.fval_2;
			params.kd = commandReceiver.fval_3;
			velocitycontrol.setParams(params);
		case SetControlLimit_vel:
			// TODO
		/*-----------------------Control Setpoint----------------------*/
		case SetControlDesired_speed:
			reactionwheelControl.setDesiredSpeed(commandReceiver.fval_1);
		case SetControlDesired_pos:
			positionControl.setDesiredAngle(commandReceiver.fval_1);
		case SetControlDesired_vel:
			velocitycontrol.setDesiredAngularVelocity(commandReceiver.fval_1);
		/*----------------------------Telemtry--------------------------*/
		case SendCalibTelemetry:
			telemetry.send_CalibIMU();
		case SendControlTelemetry:
			telemetry.send_ControlParams();
		case ToggleExtendedTelemetry:
			telemetry.enable_ExtendedTelemetry(commandReceiver.fval_1 > 0);
		/*-----------------------------Camera---------------------------*/
		// TODO
		default:
			// Skip incrementing commandCnt
			goto skipCnt;
		}

		lastCmndID = commandReceiver.id;
		this->commandCnt++;

		// marker to skip incrementing commandCnt
		skipCnt:;

	}
}

int Telecommand::getCommandCounter()
{
	return this->commandCnt;
}

int getLastCommand()
{
	return this->lastCmndID;
}

Telecommand telecommand;
