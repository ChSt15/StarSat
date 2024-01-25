#ifndef FLOATSAT_OUTERLOOP_ATTITUDEESTIMATION_HPP_
#define FLOATSAT_OUTERLOOP_ATTITUDEESTIMATION_HPP_

#include "rodos.h"
#include "matlib.h"
#include "math.h"

#include "../timestamp.hpp"
#include "IMU.hpp"

struct Attitude_Data 
{
	Quaternion attitude;
	Vector3D angularVelocity;
};

extern Topic<TimestampedData<Attitude_Data>> AttitudeDataTopic;

class QEKF
{
private:

	// State
	Vector_F<10> X;
	// Covariance of state
	Matrix_F<10, 10> P;

	//-------- Prediction ----------//
	// Jakobians of prediction
	Matrix_F<10, 10> A;
	// G * Q * G^T
	Matrix_F<10, 10> GQG;

	//-------- Correction (accel) ----------//
	// Gain
	Matrix_F<10, 3> K_a;
	// Jakobian of mesurment prediction
	Matrix_F<3, 10> C_a;
	// Covariance innovation
	Matrix_F<3, 3> S_a;
	// Covariance of measurment noise
	Matrix_F<3, 3> R_a;
	// Measurment inovation
	Vector_F<3> v_a;
	// Measurment prediction
	Vector_F<3> z_a;

	//-------- Correction (mag) ----------//
	// Gain
	Matrix_F<10, 1> K_m;
	// Jakobian of mesurment prediction
	Matrix_F<1, 10> C_m;
	// Covariance innovation
	Matrix_F<1, 1> S_m;
	// Covariance of measurment noise
	Matrix_F<1, 1> R_m;
	//Measurment inovation
	float v_yaw;
	// Measurment prediction
	float z_yaw;
	// Measurment
	float y_yaw;
	// Helper rotaions
	Matrix_F<3, 3> body2nav;
	Matrix_F<3, 3> nav2body;
	//-------- Correction (test) ----------//
	Matrix_F<10, 4> K;
	Vector_F<4> v;
	Matrix_F<4, 4> R;
	Matrix_F<4, 4> S;
	Matrix_F<4, 10> C;
	Vector_F<4> z;
	Vector_F<4> y;


	// Helper identity 10x10
	Matrix_F<10, 10> eye_10x10;

	// Last timestamp
	float last_t;

	// Attitude data
	TimestampedData <Attitude_Data> data;

	bool is_initialized = false;

public:

	QEKF();

	void config(Vector3D_F sigma_gyro, Vector3D_F sigma_accel, float sigma_yaw, float sigma_gyro_drift);

	// @brief Calculation of initial orientation
	// @param imudata -> IMU data struct defined in imu.hpp (angularVelocity [rad/s], magneticField [gauss], acceleration [g])
	void init(const TimestampedData<IMUData>& imudata);

	// @brief Orientation estimation
	// @param imudata -> IMU data struct defined in imu.hpp with timestamp of measurement.
	// @return Attitude data defined in AttitudeEstimation.hpp with timestamp of estimation.
	TimestampedData<Attitude_Data>& estimate(const TimestampedData<IMUData>& imudata);

	TimestampedData<Attitude_Data>& getestimit();

	void reset();

private:

	// @brief Propagation step using only gyro
	// @param gyro -> angluarvelocity vector in rad/s
	void propagate(const Vector3D_F& gyro, const float& time);

	// @brief Update step using accelerometer
	// @param accel -> linearacceleration vector in g
	void update_accel(const Vector3D_F& a);

	// @brief Update step using magnetometer
	// @param mag -> magneticfieldstrength vector in gauss
	void update_mag(const Vector3D_F& m);

	void update(const Vector3D_F& a, const Vector3D_F& m);
};


extern QEKF qekf;

#endif
