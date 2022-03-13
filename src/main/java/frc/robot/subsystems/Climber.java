// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class Climber extends SubsystemBase {

  // setting up the falcons for extending and retracting the climber
  private final WPI_TalonFX leftClimberMotor = new WPI_TalonFX(LEFT_CLIMBER);
  private final WPI_TalonFX rightClimberMotor = new WPI_TalonFX(RIGHT_CLIMBER);

  /** Creates a new Climber. */
  public Climber() {

    // setting the encoders for the extend retract
    leftClimberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxClimb, kTimeoutMsClimb);
    rightClimberMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxClimb, kTimeoutMsClimb);

    
    //setting all motors to brake mode
    leftClimberMotor.setNeutralMode(NeutralMode.Brake);
    rightClimberMotor.setNeutralMode(NeutralMode.Brake);
    
    
  }

  public void climbStop(){

    leftClimberMotor.set(ControlMode.PercentOutput, 0);
    rightClimberMotor.set(ControlMode.PercentOutput, 0);

  }

  
  
  public void runLeftClimber(double speed){

    leftClimberMotor.set(ControlMode.PercentOutput, speed);//need to slow down one motor

  }

  public void runRightClimber(double speed){

    rightClimberMotor.set(ControlMode.PercentOutput, speed);

  }
  
 
  // getting the current position of the left climber motor encoder
  public double getLeftClimbEncoderPosition()
  {
  
    return Math.abs(leftClimberMotor.getSelectedSensorPosition());
  
  }
  
  // getting the current position of the right climber motor encoder
  public double getRightCLimbEncoderPosition()
  {
  
    return Math.abs(rightClimberMotor.getSelectedSensorPosition());
  
  }

  //zero the extend and retract climber encoders
  public void zeroClimberEncoders()
  {
    leftClimberMotor.setSelectedSensorPosition(0);
    rightClimberMotor.setSelectedSensorPosition(0);
  }

  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
}

