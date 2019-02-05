/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//Buc'N'Gears 
//Team 3546 2019 Robot Code
//Version 0.1

#include "Robot.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <WPILib.h>
#include <Spark.h>
#include <Joystick.h>
#include <DigitalInput.h>
#include <DigitalOutput.h>
#include <Timer.h>
#include <math.h>

//Drivetrain PWM Channels
const static int leftMotorPWM = 2;
const static int rightMotorPWM = 4;
const static int strafeMotorPWM = 0;

//Climbing and Skooch Motor PWM Channels

//Drivetrain Motor Directions
const static int leftMotorDirection = -1;
const static int rightMotorDirection = 1;
const static int strafeMotorDirection = 1;

//Climbing and Skooch Motor Directions


//Joystick USB ports
const static int rightJoystickUSB = 0;
const static int leftJoystickUSB = 1;
const static int coDriverJoystickUSB = 3;

//Joystick Button Numbers
const static int strafeButton = 1;
const static int alignButton = 7;

//Drivetrain Motor Controllers
frc::Spark *left_Motor;
frc::Spark *right_Motor;
frc::Spark *strafe_Motor;

//Climbing and Skooch Motor Controllers

//Joysticks
frc::Joystick *rightJoystick;
frc::Joystick *leftJoystick;
frc::Joystick *coDriverJoystick;

//Pneumatics Control Module Definitions

//UltraSonic Senor Definitions
frc::DigitalOutput *leftPulseUltraSonicSensor;
frc::DigitalInput *leftEchoUltraSonicSensor;
frc::DigitalOutput *rightPulseUltraSonicSensor;
frc::DigitalInput *rightEchoUltraSonicSensor;

//UltraSonic Sensor Channel Numbers
const static int leftPulseUltraSonicChannel = 4;
const static int leftEchoUltraSonicChannel = 5;
const static int rightPulseUltraSonicChannel = 6;
const static int rightEchoUltraSonicChannel = 7;

//UltraSonic Sensor Numbers
const static int leftUltraSonicSensor = 1;
const static int rightUltraSonicSensor = 2;

//Line Sensor Definitions
frc::DigitalInput *rightLineSensor;
frc::DigitalInput *leftLineSensor;
frc::DigitalInput *centerLineSensor;

//Line Sensor Channel Numbers
const static int leftLineSensorChannel = 0;
const static int rightLineSensorChannel = 9;
const static int centerLineSensorChannel = 8;

// --------------------------------------------------------------------------------
// HC-SR04 ULTRASONIC SENSOR
// This method takes an argument of leftUltraSonicSensor or rightUltraSonicSensor and follows the 
//  manufacturers guidelines for obtaining a reading from the left or right
//  sensor using the pulse/echo output/input and timing the lenght of 
//  of the interval between sending and recieving the signal and then converting 
//  that (by multiplying by 17150 --  determined by Ahmad and Hannon) to something like 
//  centimeters
// --------------------------------------------------------------------------------
double MeasureForwardClearance(int SensorSide)
{
  double start_time, end_time, measuredValue;

  frc::Timer *sensorTimer;
  sensorTimer = new frc::Timer();

  sensorTimer->Reset();
  sensorTimer->Start();

  if (SensorSide == 1)	// left
  {
    leftPulseUltraSonicSensor->Set(false);
    frc::Wait(0.000010);		// wait 10uS for sensor to 'settle'

    leftPulseUltraSonicSensor->Set(true);									// send HIGH pulse to Trigger line
    //SmartDashboard::PutString("Sensor : ", "LEFT U/S TRIGGERED");
    frc::Wait(0.000010);													// keep HIGH pulse for 10 uS
    leftPulseUltraSonicSensor->Set(false);								// set Trigger to LOW

    start_time = sensorTimer->Get();
    while (leftEchoUltraSonicSensor->Get() == false && (sensorTimer->Get() - start_time) < 0.5)	// wait for sensor ECHO line to rise to HIGH
    {
      //SmartDashboard::PutString("Sensor : ", "LEFT U/S WAIT FOR ECHO");
    }

    start_time = sensorTimer->Get();
    while (leftEchoUltraSonicSensor->Get() == true && (sensorTimer->Get() - start_time) < 0.5);	// measure how long was ECHO line HIGH
    {
      //SmartDashboard::PutString("Sensor : ", "LEFT U/S MEASURING...");
    }

    //SmartDashboard::PutString("Sensor : ", "LEFT U/S COMPLETE");
    end_time = sensorTimer->Get();

    measuredValue = round((end_time - start_time) * 17150);
    frc::SmartDashboard::PutNumber("Left UltraSonic Sensor :", measuredValue);
  }

  if (SensorSide == 2)	// right
  {
    rightPulseUltraSonicSensor->Set(false);
    frc::Wait(0.000010);		// wait 10uS for sensor to 'settle'

    rightPulseUltraSonicSensor->Set(true);									// send HIGH pulse to Trigger line
    //SmartDashboard::PutString("Sensor : ", "RIGHT U/S TRIGGERED");
    frc::Wait(0.000010);														// keep HIGH pulse for 10 uS
    rightPulseUltraSonicSensor->Set(false);									// set Trigger to LOW

    start_time = sensorTimer->Get();
    while (rightEchoUltraSonicSensor->Get() == false && (sensorTimer->Get() - start_time) < 0.5)	// wait for sensor ECHO line to rise to HIGH
    {
      //SmartDashboard::PutString("Sensor : ", "RIGHT U/S WAIT FOR ECHO");
    }

    start_time = sensorTimer->Get();
    while (rightEchoUltraSonicSensor->Get() == true && (sensorTimer->Get() - start_time) < 0.5)	// measure how long was ECHO line HIGH
    {
      //SmartDashboard::PutString("Sensor : ", "RIGHT U/S MEASURING...");
    }

    //SmartDashboard::PutString("Sensor : ", "RIGHT U/S COMPLETE");
    end_time = sensorTimer->Get();

    measuredValue = round((end_time - start_time) * 17150);
    frc::SmartDashboard::PutNumber("Right UltraSonic Sensor :", measuredValue);
  }

  sensorTimer->Stop();

  return measuredValue;
}

void Robot::RobotInit() {
  //Automatically included code (next three lines)
  m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
  m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
  frc::SmartDashboard::PutData("Auto Modes", &m_chooser);

  //Motor Initialization Declarations
  left_Motor = new frc::Spark(leftMotorPWM);
  right_Motor = new frc::Spark(rightMotorPWM);
  strafe_Motor = new frc::Spark(strafeMotorPWM);

  //Joystick Initialization Declarations
  rightJoystick = new frc::Joystick(rightJoystickUSB);
  leftJoystick = new frc::Joystick(leftJoystickUSB);
  coDriverJoystick = new frc::Joystick(coDriverJoystickUSB);

  //UltraSonic Initialization Declarations
  leftPulseUltraSonicSensor = new frc::DigitalOutput(leftPulseUltraSonicChannel);
  leftPulseUltraSonicSensor -> Set(false);
  rightPulseUltraSonicSensor = new frc::DigitalOutput(rightPulseUltraSonicChannel);
  rightPulseUltraSonicSensor -> Set(false);
  leftEchoUltraSonicSensor = new frc::DigitalInput(leftEchoUltraSonicChannel);
  rightEchoUltraSonicSensor = new frc::DigitalInput(rightEchoUltraSonicChannel);

  //Line Sensor Initialization Declarations
  leftLineSensor = new frc::DigitalInput(leftLineSensorChannel);
  rightLineSensor = new frc::DigitalInput(rightLineSensorChannel);
  centerLineSensor = new frc::DigitalInput(centerLineSensorChannel);
  
}

bool LineFollowerMode()
{
  bool lineFollowerMode = rightJoystick->GetRawButton(alignButton);
  frc::SmartDashboard::PutBoolean("Pressed :", lineFollowerMode);
  return lineFollowerMode;
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want ran during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit() {
  m_autoSelected = m_chooser.GetSelected();
  // m_autoSelected = SmartDashboard::GetString(
  //     "Auto Selector", kAutoNameDefault);
  std::cout << "Auto selected: " << m_autoSelected << std::endl;

  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::AutonomousPeriodic() {
  if (m_autoSelected == kAutoNameCustom) {
    // Custom Auto goes here
  } else {
    // Default Auto goes here
  }
}

void Robot::TeleopInit() {}

void Robot::TeleopPeriodic() {
  
  bool leftLineSensorValue, rightLineSensorValue, centerLineSensorValue;;
  double leftUltraSonicSensorValue, rightUltraSonicSensorValue, error;

  //values for the alignment code 
  double total_error = 0;
  double rotation_adjustment = 0;
  bool aligned = false;

  while (IsOperatorControl() && IsEnabled())
  {
    //=========================================================================================
    //Read the joysticks and output values to the motors
    // right_Motor->Set(rightJoystick->GetY()*rightMotorDirection);
    // left_Motor->Set(leftJoystick->GetY()*leftMotorDirection);

    right_Motor->Set(1*rightMotorDirection);
    left_Motor->Set(1*leftMotorDirection);

 
    //read the trigger/strafe button and record the values
    bool strafeLeft = leftJoystick->GetRawButton(strafeButton);
    bool strafeRight = rightJoystick->GetRawButton(strafeButton);
    
    //Control the strafe driving. If both trigger/strafe buttons are depressed
    //  or both are not depressed then do nothing, otherwise only strafe left or right
    //  according to the button depressed
    if (strafeLeft && !strafeRight)   // Strafe Left
    {
        strafe_Motor->Set(0.5*strafeMotorDirection);
    }
    else if (!strafeLeft && strafeRight)    // Strafe Right
    {
        strafe_Motor->Set(-0.5*strafeMotorDirection);
    }
    else    // Stop Strafe Motor
    {
          strafe_Motor->Set(0);
    }
    //=========================================================================================

    // Alignment is only exceuted when a human presses the alignButton. LineFollowerMode() queries that
    // button and sets alignmode accordingly.
    while (LineFollowerMode())
    {
      //output to the smartdashboard
      frc::SmartDashboard::PutNumber("Squaring Up Loop :", 0);
      
      //Read the IR Sensors and output the values
      leftLineSensorValue = leftLineSensor -> Get();
      rightLineSensorValue = rightLineSensor -> Get();
      centerLineSensorValue = centerLineSensor -> Get();

      frc::SmartDashboard::PutNumber("Left Line Sensor Value :", leftLineSensorValue);
      frc::SmartDashboard::PutNumber("Right Line Sensor Value :", rightLineSensorValue);
      frc::SmartDashboard::PutNumber("Center Line Sensor Value :", centerLineSensorValue);

      //read the ultrasonic sensors initially and compute the error (the difference between them)
      leftUltraSonicSensorValue = MeasureForwardClearance(leftUltraSonicSensor);
      rightUltraSonicSensorValue = MeasureForwardClearance(rightUltraSonicSensor); 
      error = leftUltraSonicSensorValue-rightUltraSonicSensorValue;
      
      // if (error > 50 || leftUltraSonicSensorValue > 50 || rightUltraSonicSensorValue >50)
      // {
      //   frc::SmartDashboard::PutString("Too far from wall or wall not sensed :", true);
      //   break
      // }

      //We learned (in testing) that if the ultrasonic sensors are too far from the wall then this
      //  loop will overcorrect and lead to jerky behavior. 
      while (abs(error) > 2 && LineFollowerMode()) //query both the error and the alignment button so 
                                                  //a human can release the button and we exit this loop
      {
        frc::SmartDashboard::PutNumber("Squaring Up Loop :", 1);
        
        rotation_adjustment = error/10;
        
        //There seems to be a problem with the power level being to small or to big so
        // these next two if/then clause sets a maximum and a minimum
        if (rotation_adjustment > 0.5 || rotation_adjustment < -0.5)
        {
          rotation_adjustment = abs(rotation_adjustment)/rotation_adjustment*0.3;
        }

        if (rotation_adjustment > -0.2 && rotation_adjustment < 0.2)
        {
          rotation_adjustment = abs(rotation_adjustment)/rotation_adjustment*0.2;
        }

        //Output the values to the smart dashboard so that we can debug
        frc::SmartDashboard::PutNumber("Rotation Adjustment :", rotation_adjustment);

        //Set the motor power levels to adjust the alignment
        right_Motor->Set(rotation_adjustment*rightMotorDirection);
        left_Motor->Set(-rotation_adjustment*leftMotorDirection);

        frc::Wait(0.01);

        leftUltraSonicSensorValue = MeasureForwardClearance(leftUltraSonicSensor);
        rightUltraSonicSensorValue = MeasureForwardClearance(rightUltraSonicSensor); 
        error = leftUltraSonicSensorValue-rightUltraSonicSensorValue;

        if (abs(error) <= 2 || !LineFollowerMode())
        {
          right_Motor->Set(0);
          left_Motor->Set(0);
        }
      }


    }

    

    // //Error is the signed difference beween the readings of the left and right
    // // UltraSonic Sensors.  If error is positive then the left side of the 
    // // robot is further from the wall then the right side. Negative error means
    // // the opposite. 
    // //Initially query the error and if the abs(error) is less
    // // than one, the robot is considered aligned(true).
    // error = MeasureForwardClearance(leftUltraSonicSensor)-MeasureForwardClearance(rightUltraSonicSensor);
    // if (error > 1 || error < -1)
    // {
    //   aligned = false;
    // }
    // else
    // {
    //   aligned = true;
    // }

    //  //If are not aligned and in alignMode, we execute the alginment algortithm
    // //while (!aligned && alignMode)
    // while (!aligned)
    // {
    //   //Update the error and output it to the smartdashboard
    //   error = MeasureForwardClearance(leftUltraSonicSensor)-MeasureForwardClearance(rightUltraSonicSensor);
    //   frc::SmartDashboard::PutNumber("UltraSonic Sensor Difference :", error);

    //   //If the abs(error) is less than 1 then declared the robot aligned, otherwise move motors to
    //   // achieve a better alignment (i.e. a smaller error)
    //   if (error > 1 || error < -1)
    //   {
    //     //Keep track of the total error so if we need to create an PI loop (instead of a P loop) we can
    //     total_error = total_error + error;
    //     //Scale the error so to provide powerlevels for the left/right motor
    //     rotation_adjustment = error/50;
        
    //     //There seems to be a problem with the power level being to small or to big so
    //     // these next two if/then clause sets a maximum and a minimum
    //     if (rotation_adjustment > 0.4 || rotation_adjustment < -0.4)
    //     {
    //       rotation_adjustment = 0.4;
    //     }
    //     if (rotation_adjustment > -0.2 && rotation_adjustment < 0.2)
    //     {
    //       rotation_adjustment = 0.2;
    //     }

        
    //     //Output the values to the smart dashboard so that we can debug
    //     frc::SmartDashboard::PutNumber("Total Error :", total_error);
    //     frc::SmartDashboard::PutNumber("Rotation Adjustment :", rotation_adjustment);
    //     frc::SmartDashboard::PutBoolean("Alignment :", aligned);

    //     //Set the motor power levels to adjust the alignment
    //     right_Motor->Set(rotation_adjustment*rightMotorDirection);
    //     left_Motor->Set(-rotation_adjustment*leftMotorDirection);

    //     //robotDrive->MecanumDrive_Cartesian(0, 0, rotation_adjustment, ahrs->GetAngle());
    //    //frc::Wait(0.1);

    //   }
    //   else// abs(error) <1 so declare aligned, turn off motors and output values to smart dashboard
    //   {
    //     //robotDrive->MecanumDrive_Cartesian(0, 0, 0, ahrs->GetAngle());
    //     aligned = true;
    //     frc::SmartDashboard::PutBoolean("Alignment :", aligned);
    //     right_Motor->Set(0);
    //     left_Motor->Set(0);
    //   }

    //   //Finally query the alignbutton so that if a human releases the button, we exit the alignMode
    //   //alignMode = rightJoystick->GetRawButton(alignButton);

    // }
    
   

  }   

}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
