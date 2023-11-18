#ifndef FLOATSAT_CONTROL_ATTITUDEESTIMATION_HPP_
#define FLOATSAT_CONTROL_ATTITUDEESTIMATION_HPP_

#include "rodos.h"
#include "matlib.h"
#include "math.h"

#include "../timestamp.hpp"
#include "../hardware/imu.hpp"

struct Attitude_Data 
{
	Quaternion attitude;
	Vector3D angularVelocity;
};

extern Topic<TimestampedData<Attitude_Data>> AttitudeDataTopic;

class QEKF
{
private:

	// TODO: define all Matrices/vectors used

	// State
	Vector_F<4> X;
	// Covariance
	Matrix_F<4, 4> P;

	//--------Prediction
	Matrix_F<4, 4> A;
	Matrix_F<4, 3> G;
	Matrix_F<3, 3> Q;

	//--------Correction (accel)
	Matrix_F<4, 3> K_a;
	Matrix_F<3, 4> C_a;
	Matrix_F<3, 3> S_a;
	Matrix_F<3, 3> R_a;
	Vector_F<3> v_a;
	Vector_F<3> z_a;

	//--------Correction (mag)
	Matrix_F<4, 1> K_m;
	Matrix_F<1, 4> C_m;
	Matrix_F<1, 1> S_m;
	Matrix_F<1, 1> R_m;
	float v_yaw;
	float z_yaw;
	float y_yaw;

	Matrix_F<3, 3> body2nav;
	Matrix_F<3, 3> nav2body;

	// Last timestamp
	float last_t;

	TimestampedData <Attitude_Data> data;

public:

	bool is_initialized = false;

	QEKF();

	// @brief Calculation of initial orientation
	// @param imudata -> IMU data struct defined in imu.hpp (angularVelocity [rad/s], magneticField [gauss], acceleration [g])
	void init(IMUData& imudata);

	// @brief Orientation estimation
	// @param imudata -> IMU data struct defined in imu.hpp with timestamp of measurement.
	// @return Attitude data defined in AttitudeEstimation.hpp with timestamp of estimation.
	TimestampedData<Attitude_Data>& estimate(TimestampedData<IMUData>& imudata);

private:

	// @brief Propagation step using only gyro
	// @param gyro -> angluarvelocity vector in rad/s
	void propagate(Vector3D_F gyro);

	// @brief Update step using accelerometer
	// @param accel -> linearacceleration vector in g
	void update_accel(Vector3D_F a);

	// @brief Update step using magnetometer
	// @param mag -> magneticfieldstrength vector in gauss
	void update_mag(Vector3D_F m);

};


extern QEKF qekf;

#endif
