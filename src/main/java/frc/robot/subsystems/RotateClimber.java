// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.AnalogInput;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RotateClimber extends SubsystemBase {
  /** Creates a new RotateClimber. */
  
  
  // setting up the falcons for rotating the climber
  private final WPI_TalonFX leftRotateMotor = new WPI_TalonFX(LEFT_ROTATE_CLIMBER);
  private final WPI_TalonFX rightRotateMotor = new WPI_TalonFX(RIGHT_ROTATE_CLIMBER);

  

  // setting up the absolute encoders for climber angle
  private final AnalogInput leftAngleEncoder = new AnalogInput(LEFT_CLIMBER_ANGLE);
  private final AnalogInput rightAngleEncoder = new AnalogInput(RIGHT_CLIMBER_ANGLE);

  
  public RotateClimber() {

  
  // setting the encoders for the rotating motors
  leftRotateMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxClimb, kTimeoutMsClimb);
  rightRotateMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxClimb, kTimeoutMsClimb);

  //setting all motors to brake mode

  leftRotateMotor.setNeutralMode(NeutralMode.Brake);
  rightRotateMotor.setNeutralMode(NeutralMode.Brake);
 

  }


  public void rotateClimbStop(){
  
    leftRotateMotor.set(ControlMode.PercentOutput, 0);
    rightRotateMotor.set(ControlMode.PercentOutput, 0);
  
  }
  
  

  public void rotateLeft(double speed)
  {
    leftRotateMotor.set(ControlMode.PercentOutput, speed);


  }
  

  public void rotateRight(double speed)
  {
    rightRotateMotor.set(ControlMode.PercentOutput, speed);
  }


  
  // getting the current position of the left rotate motor encoder
  public double getLeftRotateEncoderPosition()
  {

    return Math.abs(leftRotateMotor.getSelectedSensorPosition());

  }

  // getting the current position of the right rotate motor encoder 
  public double getRightRotateEncoderPosition()
  {

    return Math.abs(rightRotateMotor.getSelectedSensorPosition());

  }

  //zero the extend and retract climber encoders
  public void zeroRotateClimberEncoders()
  {
    leftRotateMotor.setSelectedSensorPosition(0);
    rightRotateMotor.setSelectedSensorPosition(0);
  }

  // getting the current left climber angle
  public double getLeftClimberAngle()
  {

    return (leftAngleEncoder.getVoltage()/5.0)*360.0;

  }

  public double getRightClimberAngle()
  {

    return (rightAngleEncoder.getVoltage()/5.0)*360.0;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
