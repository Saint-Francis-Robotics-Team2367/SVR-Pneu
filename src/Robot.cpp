#include "WPILib.h"
#include "pthread.h"
#include "SfDrive.hpp"
#define FL_ID 3
#define FR_ID 2
#define RL_ID 1
#define RR_ID 4

#define LIFTER_FCHAN 1
#define LIFTER_RCHAN 0

#define L_INTAKE_CONST 1
#define R_INTAKE_CONST 1


class Robot: public IterativeRobot
{
	Joystick *firstDriver; // only joystick
	CANTalon *frontLeftMot;
	CANTalon *frontRightMot;
	CANTalon *rearLeftMot;
	CANTalon *rearRightMot;

	CANTalon *leftIntake;
	CANTalon *rightIntake;

	DoubleSolenoid *lifter;
	SfDrive *driveTrain;

	DriverStation *ds;

public:
	Robot()
	{
		this->frontLeftMot = new CANTalon(FL_ID);
		this->frontRightMot = new CANTalon(FR_ID);
		this->rearLeftMot = new CANTalon(RL_ID);
		this->rearRightMot = new CANTalon(RR_ID);

		this->leftIntake = new CANTalon(5);
		this->rightIntake = new CANTalon(6);

		this->lifter = new DoubleSolenoid(LIFTER_FCHAN, LIFTER_RCHAN);

		this->firstDriver = new Joystick(0);

		this->driveTrain = new SfDrive(this, this->firstDriver, new Gyro(1), frontLeftMot,
				frontRightMot, rearLeftMot, rearRightMot);


		this->ds = DriverStation::GetInstance();
	}

private:
	void RobotInit()
	{

	}

	void AutonomousInit()
	{

	}

	void AutonomousPeriodic()
	{
		float lastTime = Timer::GetFPGATimestamp();
		while (Timer::GetFPGATimestamp() - lastTime < 5)
		{
			this->driveTrain->setLeftRightMots(0.5, 0.5);
		}
		while (IsAutonomous() && IsEnabled())
		{
			this->driveTrain->setLeftRightMots(0, 0);
		}
	}

	void TeleopInit()
	{
		this->driveTrain->startMechanumDrive();
	}

	void TeleopPeriodic()
	{
		if (this->firstDriver->GetRawButton(6))
			this->lifter->Set(DoubleSolenoid::kForward);
		else if (this->firstDriver->GetRawButton(5))
			this->lifter->Set(DoubleSolenoid::kReverse);
		this->leftIntake->Set(this->firstDriver->GetRawAxis(3)/L_INTAKE_CONST);
		this->rightIntake->Set(this->firstDriver->GetRawAxis(4)/R_INTAKE_CONST);

	}
	void TestPeriodic()
	{

	}
	void DisabledPeriodic()
	{
		this->driveTrain->setLeftRightMots(0, 0);
		this->leftIntake->Set(0);
		this->rightIntake->Set(0);
	}

};

START_ROBOT_CLASS(Robot);

