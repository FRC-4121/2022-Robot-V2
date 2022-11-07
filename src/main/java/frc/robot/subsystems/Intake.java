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

  // Declare motors
  private WPI_TalonSRX intakeMain = new WPI_TalonSRX( INTAKE);
  private WPI_TalonFX intakeRelease = new WPI_TalonFX(INTAKERELEASE);


  /** Creates a new Intake. */
  public Intake() {

        // Setting the encoders for the intake release
        intakeRelease.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxClimb, kTimeoutMsClimb);

  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /* Method to run the motor for the intake wheels */
  public void runIntake()
  {
    intakeMain.set(-0.5);
  }
 
  /* Method to stop the motor for the intake wheels */
  public void stopIntake()
  {
      intakeMain.set(0);
  }

  /* Run the intake release motor (raise or lower intake) */
  public void runIntakeRelease(double speed)
  {

    intakeRelease.set(speed);

  }

  /* Method to stop the motor for the intake release */
  public void intakeReleaseStop()
  {

    //sets motor speed to stop
    intakeRelease.set(0);

  }

  /* getting the current position of the left climber motor encoder */
  public double getIntakeReleaseEncoderPosition()
  {
    
    return intakeRelease.getSelectedSensorPosition();
    
  }
    
  //zero the intake encoder
  public void zeroIntakeEncoder()
  {
    intakeRelease.setSelectedSensorPosition(0);
  }

}
