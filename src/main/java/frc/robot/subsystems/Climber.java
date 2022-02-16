// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber extends SubsystemBase {

  // setting up the falcons for extending and retracting the climber
  private final WPI_TalonFX leftClimberMotor = new WPI_TalonFX(LEFT_CLIMBER);
  private final WPI_TalonFX rightClimberMotor = new WPI_TalonFX(RIGHT_CLIMBER);

  // setting up the falcons for rotating the climber
  private final WPI_TalonFX leftRotateMotor = new WPI_TalonFX(LEFT_ROTATE_CLIMBER);
  private final WPI_TalonFX rightRotateMotor = new WPI_TalonFX(RIGHT_ROTATE_CLIMBER);

  /** Creates a new Climber. */
  public Climber() {

    // setting the encoders for the extend retract
    leftClimberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxClimb, kTimeoutMsClimb);
    rightClimberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxClimb, kTimeoutMsClimb);
    leftClimberMotor.setSelectedSensorPosition(0);
    rightClimberMotor.setSelectedSensorPosition(0);

    // setting the encoders for the rotating motors
    leftRotateMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxClimb, kTimeoutMsClimb);
    rightRotateMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxClimb, kTimeoutMsClimb);
    leftRotateMotor.setSelectedSensorPosition(0);
    rightRotateMotor.setSelectedSensorPosition(0);
    
  }

  public void climbStop(){

    leftClimberMotor.set(ControlMode.PercentOutput, 0);
    rightClimberMotor.set(ControlMode.PercentOutput, 0);

  }

  public void climbExtend(double speed){
    
    // setting the speeds for extending
    leftClimberMotor.set(ControlMode.PercentOutput, -speed * ClimberLimiter);//need to slow down one motor
    rightClimberMotor.set(ControlMode.PercentOutput, speed);
  
  }
  
  public void climbRetract(double speed){
           
    // setting the speeds for retracting
    leftClimberMotor.set(ControlMode.PercentOutput, speed * ClimberLimiter);//need to slow down one motor
    rightClimberMotor.set(ControlMode.PercentOutput, -speed);
  
  }
  
  public void rotateClimbStop(){
  
    leftRotateMotor.set(ControlMode.PercentOutput, 0);
    rightRotateMotor.set(ControlMode.PercentOutput, 0);
  
  }
  
  public void rotateClimbOut(double speed){
      
    // setting the speeds for rotating outwards
    leftRotateMotor.set(ControlMode.PercentOutput, speed);
    rightRotateMotor.set(ControlMode.PercentOutput, speed);
  
  }
  
  public void rotateClimbIn(double speed){
           
    // setting the speeds for rotating inwards
    leftRotateMotor.set(ControlMode.PercentOutput, speed);
    rightRotateMotor.set(ControlMode.PercentOutput, speed);
  
  }
  
  // getting the current position of the left climber motor encoder
  public double getLeftClimbEncoderPosition()
  {
  
    return leftClimberMotor.getSelectedSensorPosition();
  
  }
  
  // getting the current position of the right climber motor encoder
  public double getRightCLimbEncoderPosition()
  {
  
    return rightClimberMotor.getSelectedSensorPosition();
  
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}

