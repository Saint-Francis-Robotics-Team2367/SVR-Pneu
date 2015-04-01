/*
 * SfDrive.hpp
 *
 *  Created on: Mar 19, 2015
 *      Author: daniel
 */


#ifndef SRC_SFDRIVE_HPP_
#define SRC_SFDRIVE_HPP_

#define MAX_MOTORS 4

#include "WPILib.h"
#include "Math.h"

class SfDrive
{
public:
	enum MotType{TalonSRX, Jaguar, Talon, Victor};
	SfDrive(IterativeRobot *rob, Joystick *stick, Gyro *gyro, CANTalon *frontLeftMot, CANTalon *frontRightMot, CANTalon *rearLeftMot,CANTalon *rearRightMot);

	void startMechanumDrive();
	void setLeftRightMots(float left, float right);
	void moveDistance(float speed, float distance);//move a specified distance
	void turnAngle(float speed, float angle);
	void turnAngle(float speed, float angle, float timeOut);
	void mechanumDrive(float x, float y, float rot);
	void *mechDriver(void *data);
private:
	enum MotPos{frontLeft = 0, frontRight = 1, rearLeft=2, rearRight = 3};
	CANTalon *frontLeftMot;
	CANTalon *frontRightMot;
	CANTalon *rearLeftMot;
	CANTalon *rearRightMot;

	IterativeRobot *robot;

	Gyro *gyro;
	Joystick *driveStick;

	pthread_t driveThread;


	void normalizeMotSpeeds();
	bool deadZone(float x);

	float kFL, kFR, kRL,kRR;
	float motSpeeds[MAX_MOTORS];
	int invertedMots[MAX_MOTORS];
};

#endif /* SRC_SFDRIVE_HPP_ */
