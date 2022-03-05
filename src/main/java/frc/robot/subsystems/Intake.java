// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


public class Intake extends SubsystemBase {

  //motors
  private WPI_TalonSRX intakeMain = new WPI_TalonSRX( INTAKE);
  private WPI_TalonFX intakeRelease = new WPI_TalonFX(INTAKERELEASE);


  /** Creates a new Intake. */
  public Intake() {

        // setting the encoders for the intake release
        intakeRelease.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxClimb, kTimeoutMsClimb);
        intakeRelease.setSelectedSensorPosition(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // method to run the motor for the intake wheels
  public void runIntake()
  {
    intakeMain.set(-0.35);
  }
 

  //method to stop the motor for the intake wheels
  public void stopIntake()
  {
      intakeMain.set(0);
  }


  //method to start the motor for the intake release
  public void intakeLower()
  {
      intakeRelease.set(0.1);
  }

  //method to start the motor for the intake release
  public void intakeRaise()
  {
      intakeRelease.set(-0.1);
  }


  //method to stop the motor for the intake release 
  public void intakeReleaseStop()
  {
    //sets motor speed to stop
    intakeRelease.set(0);
  }

    // getting the current position of the left climber motor encoder
    public double getIntakeReleaseEncoderPosition()
    {
    
      return intakeRelease.getSelectedSensorPosition();
    
    }
    

}
