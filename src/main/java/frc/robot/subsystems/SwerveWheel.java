// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.kinematics.*;
import static frc.robot.Constants.*;
import static frc.robot.Constants.DrivetrainConstants.*;
import frc.robot.ExtraClasses.*;

public class SwerveWheel extends SubsystemBase { 

  /* Declare motor variables */
  private WPI_TalonFX swerveDriveMotor;
  private WPI_TalonFX swerveAngleMotor;

  /* Declare controller variables */
  private PIDControl drivePIDController;
  private PIDControl anglePIDController;

  /* Declare state variables */
  private double wheelSpeed;
  private double wheelAngle;

  /* Creates a new SwerveWheel. */
  public SwerveWheel(int driveMotorID, int angleMotorID) {

    // Initialize the swerve motors
    InitSwerveMotors(driveMotorID, angleMotorID);

    // Initialize PID controllers
    drivePIDController = new PIDControl(kP_SwerveDriveSpeed, kI_SwerveDriveSpeed, kD_SwerveDriveSpeed);
    anglePIDController = new PIDControl(kP_SwerveDriveAngle, kI_SwerveDriveAngle, kD_SwerveDriveAngle);

  }

  /* Initialize motors */
  private void InitSwerveMotors(int driveMotorID, int angleMotorID) {

    // Create motors
    swerveDriveMotor = new WPI_TalonFX(driveMotorID);
    swerveAngleMotor = new WPI_TalonFX(angleMotorID);

    // Set brake mode
    swerveDriveMotor.setNeutralMode(NeutralMode.Brake);
    swerveAngleMotor.setNeutralMode(NeutralMode.Brake);

    // Set motor encoders
    swerveDriveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
    swerveAngleMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdxDrive, kTimeoutMsDrive);
  
  }
 
  /* Periodically set wheel speed and angle */
  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  /* Takes a module state and sets the velocity and angle */
  public void SetDrive(SwerveModuleState driveState) {

  }

}
