// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.*;

import static frc.robot.Constants.*;
import frc.robot.ExtraClasses.PIDControl;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.CommandBase;


public class AutoDrive extends CommandBase {

  /** Creates a new AutoDrive. */
  private final Drivetrain drivetrain;
  private double targetDriveDistance; //inches
  private double targetAngle;
  private double direction;
  private double stopTime;
  private double currentGyroAngle = 0;
  private double driveDirection;
  private double driveSpeed;

  private double angleCorrection, speedCorrection;
  private double startTime;
  private double distanceTraveled;

  private double leftEncoderStart;
  private double rightEncoderStart;
  private double totalRotationsLeft = 0;
  private double totalRotationsRight = 0;


  private Timer timer = new Timer();
  private PIDControl pidDriveAngle;
  private PIDControl pidDriveDistance; 

  //distance is in inches

  public AutoDrive(Drivetrain drive, double dis, double ang, double dir, double time) {
   
    targetDriveDistance = dis;
    targetAngle = ang;
    direction = dir;
    stopTime = time;
    drivetrain = drive;

    addRequirements(drivetrain);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    distanceTraveled = 0.0;
    timer.start();
    startTime = timer.get();

    // Zero the gyro and encoders
    drivetrain.zeroGyro();
    drivetrain.zeroEncoders();

    leftEncoderStart = drivetrain.getMasterLeftEncoderPosition();
    rightEncoderStart = drivetrain.getMasterRightEncoderPosition();

    angleCorrection = 0;
    speedCorrection = 1;

    //The constants for these need to be figured out
    pidDriveAngle = new PIDControl(kP_DriveAngle, kI_DriveAngle, kD_DriveAngle);
    pidDriveDistance = new PIDControl(kP_Straight, kI_Straight, kD_Straight);

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Calculate angle correction based on gyro reading
    currentGyroAngle = drivetrain.getGyroAngle();
    angleCorrection = pidDriveAngle.run(currentGyroAngle, targetAngle); //the speed that the robot rotates from currentGyroAngle to targetAngle between -1 to 1
    // SmartDashboard.putNumber("AngleCor", angleCorrection);

    // Calculate speed correction based on distance to target
    // totalRotationsRight = Math.abs((Math.abs(drivetrain.getMasterRightEncoderPosition()) - rightEncoderStart));
    // totalRotationsLeft = Math.abs((Math.abs(drivetrain.getMasterLeftEncoderPosition()) - leftEncoderStart));
    // distanceTraveled = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / (DrivetrainConstants.kTalonFXPPR * kGearRatio);
    // speedCorrection = pidDriveDistance.run(distanceTraveled, targetDriveDistance); //the closer, the slower robot move. The further, the faster the robot moves
    // SmartDashboard.putNumber("DrSpCorrection", speedCorrection);

    //Check for overspeed correction
    // if(speedCorrection > 1){

    //    speedCorrection = 1;

    // } else if (speedCorrection < -1) {

    //   speedCorrection = -1;

    // }


    driveSpeed = direction * speedCorrection * kAutoDriveSpeed;

    SmartDashboard.putNumber("DriveSpeed", driveSpeed);
    // Enforce minimum speed
    // if (Math.abs(driveSpeed) < kAutoDriveSpeedMin) {
    //   angleCorrection = 0;
    //   if (driveSpeed < 0){
    //     driveSpeed = -kAutoDriveSpeedMin;
    //   } else {
    //     driveSpeed = kAutoDriveSpeedMin;
    //   }
    // }
    SmartDashboard.putNumber("AngleCor", angleCorrection);

    double leftSpeed = 0;
    double rightSpeed = 0;
    if (Math.abs(driveSpeed + angleCorrection) > 1){
      if(driveSpeed + angleCorrection < 0) {//do this because angleCorrection is sometimes over 1 so hard code it to -1 or 1
        leftSpeed = -1;
      } else {
        leftSpeed = 1;
      }
    } else {
      leftSpeed = driveSpeed + angleCorrection;
    }

    if (Math.abs(driveSpeed - angleCorrection) > 1){
      if(driveSpeed - angleCorrection < 0) { //do this because angleCorrection is sometimes over 1 so hard code it to -1 or 1
        rightSpeed = -1;
      } else {
        rightSpeed = 1;
      }
    } else {
      rightSpeed = driveSpeed - angleCorrection;
    }
    rightSpeed *= kSpeedCorrection; //this may need to be used on the left drive motors
    SmartDashboard.putNumber("LeftSpeed", leftSpeed);
    SmartDashboard.putNumber("RightSpeed", rightSpeed);

    // Run the drive
    drivetrain.autoDrive(leftSpeed, rightSpeed); //from which method does this come from.

    double totalRotationsRight = Math.abs((drivetrain.getMasterRightEncoderPosition() - rightEncoderStart)); //does getEncoderPosition return rotations or another unit?
    double totalRotationsLeft = Math.abs((drivetrain.getMasterLeftEncoderPosition() - leftEncoderStart));

    distanceTraveled = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / AUTO_ENCODER_REVOLUTION_FACTOR;
    SmartDashboard.putNumber("Distance", distanceTraveled);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.stopDrive();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean thereYet = false;
    double time = timer.get();

    // Check distance against target
    SmartDashboard.putNumber("Distance error", Math.abs(distanceTraveled - targetDriveDistance));
    if (distanceTraveled >= targetDriveDistance) {
      thereYet = true;
    } else if (time - startTime >= stopTime) {
      thereYet = true;
    } else if (killAuto == true) {
      thereYet = true;
    }

    return thereYet;
  }

}

