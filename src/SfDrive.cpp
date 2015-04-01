/*
 * SfDrive.cpp
 *
 *  Created on: Mar 19, 2015
 *      Author: daniel
 */

#include <SfDrive.hpp>




static void *mechHelper(void *data)
{
	DriverStation::ReportError("ENTERING THREAD\n");
	return ((SfDrive *) data)->mechDriver(data);
}


SfDrive::SfDrive(IterativeRobot *rob, Joystick *stick, Gyro *gyro, CANTalon *frontLeftMot, CANTalon *frontRightMot, CANTalon *rearLeftMot,CANTalon *rearRightMot)
{
	this->robot = rob;
	this->driveStick = stick;
	this->gyro = gyro;

	this->frontLeftMot = frontLeftMot;
	this->frontRightMot = frontRightMot;
	this->rearLeftMot = rearLeftMot;
	this->rearRightMot = rearRightMot;

	this->frontLeftMot->SetFeedbackDevice(CANTalon::QuadEncoder);
	this->frontRightMot->SetFeedbackDevice(CANTalon::QuadEncoder);
	this->rearLeftMot->SetFeedbackDevice(CANTalon::QuadEncoder);
	this->rearRightMot->SetFeedbackDevice(CANTalon::QuadEncoder);


	this->driveThread = 1;

	this->kFL = 1;
	this->kFR = 1;
	this->kRL = 1;
	this->kRR = 1;
	this->invertedMots[frontLeft] = 1;
	this->invertedMots[frontRight] = 1;
	this->invertedMots[rearLeft] = 1;
	this->invertedMots[rearRight] = 1;
}




void SfDrive::startMechanumDrive()
{
	int tele;
	void *drive = this;
	tele = pthread_create(&driveThread, NULL, mechHelper, drive);
	if(!tele) DriverStation::ReportError("THREAD IS GOOD\n");
	else DriverStation::ReportError("THREAD IS BAD\n");

}


void SfDrive::mechanumDrive(float x, float y, float rot)
{
	y = -y;
	this->motSpeeds[frontLeft] = x + y + rot;
	this->motSpeeds[frontRight] = -x + y - rot;
	this->motSpeeds[rearLeft] = -x + y + rot;
	this->motSpeeds[rearRight] = x + y - rot;
	this->normalizeMotSpeeds();



	this->frontLeftMot->Set(this->motSpeeds[frontLeft] * this->invertedMots[frontLeft] * kFL);
	this->frontRightMot->Set(this->motSpeeds[frontRight] * this->invertedMots[frontRight] * kFR);
	this->rearLeftMot->Set(this->motSpeeds[rearLeft] * this->invertedMots[rearLeft] * kRL);
	this->rearRightMot->Set(this->motSpeeds[rearRight] * this->invertedMots[rearRight] * kRR);
}

void SfDrive::setLeftRightMots(float left, float right)
{
	if (frontLeftMot != NULL)
	{
		frontLeftMot->Set(left);
	}
	if (rearLeftMot != NULL)
	{
		rearLeftMot->Set(left);
	}
	if (frontRightMot != NULL)
	{
		frontRightMot->Set(right);
	}
	if (rearRightMot != NULL)
	{
		rearRightMot->Set(right);
	}
}

void SfDrive::normalizeMotSpeeds()
{
	float maxVel = fabs(this->motSpeeds[0]);
	for (int i = 0; i < MAX_MOTORS; i++)
	{
		if (fabs(this->motSpeeds[i]) > maxVel)
		{
			maxVel = fabs(this->motSpeeds[i]);
		}
	}
	if (maxVel > 1)
	{
		for (int i = 0; i < MAX_MOTORS; i++)
		{
			this->motSpeeds[i] = this->motSpeeds[i] / maxVel;
		}
	}
}


void SfDrive::moveDistance(float speed, float dist)
{
	this->frontLeftMot->SetControlMode(CANTalon::kPosition);
	this->frontRightMot->SetControlMode(CANTalon::kPosition);
	this->rearLeftMot->SetControlMode(CANTalon::kPosition);
	this->rearRightMot->SetControlMode(CANTalon::kPosition);


	this->frontLeftMot->SetPID(1.8,0,0);
	this->frontRightMot->SetPID(1.8,0,0);
	this->rearLeftMot->SetPID(1.8,0,0);
	this->rearRightMot->SetPID(1.8,0,0);

	this->frontLeftMot->SetPosition(0);
	this->frontRightMot->SetPosition(0);
	this->rearLeftMot->SetPosition(0);
	this->rearRightMot->SetPosition(0);

	double posAvg  = (this->frontLeftMot->GetPosition()+
			this->frontRightMot->GetPosition()+
			this->rearLeftMot->GetPosition()+
			this->rearRightMot->GetPosition())/4;
	while(fabs(posAvg-dist) > 50)
	{
		{
			posAvg = (this->frontLeftMot->GetPosition()+
			this->frontRightMot->GetPosition()+
			this->rearLeftMot->GetPosition()+
			this->rearRightMot->GetPosition())/4;
		}

		this->frontLeftMot->Set(dist);
		this->frontRightMot->Set(dist);
		this->rearLeftMot->Set(dist);
		this->rearRightMot->Set(dist);
	}

	this->frontLeftMot->SetControlMode(CANTalon::kPercentVbus);
	this->frontRightMot->SetControlMode(CANTalon::kPercentVbus);
	this->rearLeftMot->SetControlMode(CANTalon::kPercentVbus);
	this->rearRightMot->SetControlMode(CANTalon::kPercentVbus);
}

void SfDrive::turnAngle(float speed, float angle)
{
	this->gyro->Reset();
	if(angle<0)
	{
		speed = -speed;
	}
	while(fabs(this->gyro->GetAngle())<angle)
	{
		this->frontLeftMot->Set(speed);
		this->frontRightMot->Set(speed);
		this->rearLeftMot->Set(speed);
		this->rearRightMot->Set(speed);
	}

}

void SfDrive::turnAngle(float speed, float angle,float timeOut)
{
	this->gyro->Reset();
	if(angle<0)
	{
		speed = -speed;
	}
	float lastTime = Timer::GetFPGATimestamp();
	while(fabs(this->gyro->GetAngle())<angle && (Timer::GetFPGATimestamp()-lastTime)<timeOut)
	{
		this->frontLeftMot->Set(speed);
		this->frontRightMot->Set(speed);
		this->rearLeftMot->Set(speed);
		this->rearRightMot->Set(speed);
	}

}

void *SfDrive::mechDriver(void *data)
{
	DriverStation::ReportError("IN THREAD\n");
	SfDrive *drive = (SfDrive *) data;
	while (drive->robot->IsEnabled() && drive->robot->IsOperatorControl())
	{
		if (drive->deadZone(drive->driveStick->GetRawAxis(0))
				&& drive->deadZone(drive->driveStick->GetRawAxis(1))
				&& drive->deadZone(drive->driveStick->GetRawAxis(4)))
		{
			drive->setLeftRightMots(0, 0);
		}
		else
			drive->mechanumDrive(
					drive->driveStick->GetRawAxis(0) * -1,
					drive->driveStick->GetRawAxis(1) * -1,
					drive->driveStick->GetRawAxis(4) * -1);
		Wait(0.005);
	}
	return data;

}

bool SfDrive::deadZone(float x)
{
	if (fabs(x) < 0.1)
	return true;
	else
	return false;
}
