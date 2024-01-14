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
			calib.scale = Matrix3D_F(Vector3D_F(1, 0, 0), Vector3D_F(0, 1, 0), Vector3D_F(0, 0, 1));
			imu.setGyroCalib(calib);
			break;
		case SetCalibParams_accel:
			calib.bias.x = commandReceiver.fval_1;
			calib.bias.y = commandReceiver.fval_2;
			calib.bias.z = commandReceiver.fval_3;
			calib.scale = Matrix3D_F(Vector3D_F(1, 0, 0), Vector3D_F(0, 1, 0), Vector3D_F(0, 0, 1));
			imu.setGyroCalib(calib);
			break;
		case SetCalibParams_mag:
			calib.bias.x = commandReceiver.fval_1;
			calib.bias.y = commandReceiver.fval_2;
			calib.bias.z = commandReceiver.fval_3;
			calib.scale = Matrix3D_F(Vector3D_F(1, 0, 0), Vector3D_F(0, 1, 0), Vector3D_F(0, 0, 1));
			imu.setGyroCalib(calib);
			break;
		/*-------------------------Control Parms------------------------*/
		case SetControlParams_speed:
			params.kp = commandReceiver.fval_1;
			params.ki = commandReceiver.fval_2;
			params.kd = commandReceiver.fval_3;
			reactionwheelControl.setParams(params);
			break;
		case SetControlLimit_speed:
			reactionwheelControl.setLimit(commandReceiver.fval_1);
			break;
		case SetControlParams_pos:
			params.kp = commandReceiver.fval_1;
			params.ki = commandReceiver.fval_2;
			params.kd = commandReceiver.fval_3;
			positionControl.setParams(params);
			break;
		case SetControlLimit_pos:
			positionControl.setLimit(commandReceiver.fval_1);
			break;
		case SetControlParams_vel:
			params.kp = commandReceiver.fval_1;
			params.ki = commandReceiver.fval_2;
			params.kd = commandReceiver.fval_3;
			velocitycontrol.setParams(params);
			break;
		case SetControlLimit_vel:
			velocitycontrol.setLimit(commandReceiver.fval_1);
			break;
		/*-----------------------Control Setpoint----------------------*/
		case SetControlDesired_speed:
			reactionwheelControl.setSetpoint(commandReceiver.fval_1);
			break;
		case SetControlDesired_pos:
			positionControl.setSetpoint(commandReceiver.fval_1);
			break;
		case SetControlDesired_vel:
			velocitycontrol.setSetpoint(commandReceiver.fval_1);
			break;
		/*----------------------------Telemtry--------------------------*/
		case SendCalibTelemetry:
			telemetry.send_CalibIMU();
			break;
		case SendControlTelemetry:
			telemetry.send_ControlParams();
			break;
		case ToggleExtendedTelemetry:
			telemetry.enable_ExtendedTelemetry(commandReceiver.fval_1 > 0);
			break;
		/*-----------------------------Camera---------------------------*/
		// TODO
		default:
			// Skip incrementing commandCnt
			continue;
		}

		lastCmndID = commandReceiver.id;
		this->commandCnt++;
	}
}

int Telecommand::getCommandCounter()
{
	return this->commandCnt;
}

int Telecommand::getLastCommand()
{
	return this->lastCmndID;
}

Telecommand telecommand;
