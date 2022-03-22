/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import static frc.robot.Constants.DrivetrainConstants.*;
import frc.robot.ExtraClasses.PIDControl;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.subsystems.Processor;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;

public class AutoPickUpBall extends CommandBase {
  
  private final Drivetrain drivetrain;
  private final Processor processor;
  private final NetworkTableQuerier ntables;
  private final Intake intake;

  private double targetDistance;
  private double targetAngle;
  private double stopTime;

  private double angleCorrection, angleError, speedCorrection;
  private double startTime;
  private double distanceTraveled;

  private double targetGyro;
  private double actualGyro;
  private boolean holdGyro;

  private double leftEncoderStart;
  private double rightEncoderStart;

  private boolean isBallOnBoard;

  private Timer timer;
  private PIDControl pidAngle;
  private boolean runProcessor;


  public AutoPickUpBall(Drivetrain drive, Processor process, Intake in, NetworkTableQuerier table, double time) {

    processor = process;
    drivetrain = drive;
    ntables = table;
    intake = in;
    addRequirements(drivetrain, processor);

    stopTime = time;
    timer = new Timer();
    pidAngle = new PIDControl(kP_Turn, kI_Turn, kD_Turn);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.start();
    startTime = timer.get();

    angleCorrection = 0;
    angleError = 0;
    speedCorrection = 1;

    drivetrain.zeroGyro();
    drivetrain.zeroEncoders();

    leftEncoderStart = drivetrain.getMasterLeftEncoderPosition();
    rightEncoderStart = drivetrain.getMasterRightEncoderPosition();

    holdGyro = false;
    targetGyro = drivetrain.getGyroAngle();
    actualGyro = drivetrain.getGyroAngle();

    isBallOnBoard = true;
    runProcessor = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    processor.runProcessor(0.1);
    intake.runIntake();

    // Get vision values
    double ballOffset = ntables.getVisionDouble("BallOffset0") + kCameraCorrection;
    double ballDistance = ntables.getVisionDouble("BallDistance0");
    boolean foundBall = ntables.getVisionBoolean("FoundBall");
    SmartDashboard.putBoolean("FoundBAll", foundBall);
    SmartDashboard.putNumber("ballDistance", ballDistance);
    SmartDashboard.putNumber("ballOffset", ballOffset);

    // Read current gyro angle
    actualGyro = drivetrain.getGyroAngle();

    // Update target gyro angle unless we are holding
    if (holdGyro == false){
      targetGyro = actualGyro;
    }
    
    // Calculate correction based on ball offset if far away
    // or based on gyro angle if close enough
    if (holdGyro == false){
      angleCorrection = pidAngle.run(ballOffset, 0);
    } else {
      angleCorrection = pidAngle.run(actualGyro, targetGyro);
    }
    
    // Calculate speed correction based on distance
    if ((ballDistance > 50) && foundBall == true){
      speedCorrection = 1;
    }else{
      speedCorrection = .6;
    }

    // Run drive train
    if (foundBall) 
    {

      // Calculate left side drive speed 
      double leftDriveSpeed = (speedCorrection * kAutoDriveSpeed) - angleCorrection;
      if (leftDriveSpeed > 1)
      {
        leftDriveSpeed = 1;
      }
      else if (leftDriveSpeed < -1)
      {
        leftDriveSpeed = -1;
      }

      // Calculate right side drive speed
      double rightDriveSpeed = (speedCorrection * kAutoDriveSpeed) + angleCorrection;
      if (rightDriveSpeed > 1)
      {
        rightDriveSpeed = 1;
      }
      else if (rightDriveSpeed < -1)
      {
        rightDriveSpeed = -1;
      }
      
      // Drive the bot
      drivetrain.autoDrive(-leftDriveSpeed, -rightDriveSpeed);

      // Put values on dashboard for testing
      SmartDashboard.putNumber("Left Drive Speed", leftDriveSpeed);
      SmartDashboard.putNumber("Right Drive Speed", rightDriveSpeed);
      SmartDashboard.putNumber("SpeedCorrection",speedCorrection);
      SmartDashboard.putNumber("kAutoDriveSpeed",kAutoDriveSpeed);
      SmartDashboard.putNumber("angleCorrection",angleCorrection);
      SmartDashboard.putNumber("SpeedCorrect", speedCorrection);
      SmartDashboard.putNumber("ActualGyro", actualGyro);
      SmartDashboard.putNumber("TargetGyro", targetGyro);
      SmartDashboard.putNumber("Angle Correct", angleCorrection);
  
    } 
    else 
    {

      //drivetrain.stopDrive();//modified for showcase purposes

    }

    SmartDashboard.putNumber("Angle Correction", angleCorrection);

    double totalRotationsRight = Math.abs((drivetrain.getMasterRightEncoderPosition() - rightEncoderStart)); //does getEncoderPosition return rotations or another unit?
    double totalRotationsLeft = Math.abs((drivetrain.getMasterLeftEncoderPosition() - leftEncoderStart));

    distanceTraveled = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / AUTO_ENCODER_REVOLUTION_FACTOR;
    SmartDashboard.putNumber("Distance", distanceTraveled);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Stop all motors
    drivetrain.stopDrive();
    processor.stopProcessor();
    intake.stopIntake();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean thereYet = false;

    double time = timer.get();

    isBallOnBoard = true;
    isBallOnBoard = processor.getIntakeSwitch();
 
    SmartDashboard.putNumber("Auto Time", time);
    SmartDashboard.putNumber("Auto Start Time", startTime);
    SmartDashboard.putBoolean("BallOnBoard", isBallOnBoard);

    if(isBallOnBoard == false) {
      runProcessor = true;
      drivetrain.stopDrive();
    }
    else if (isBallOnBoard)
    {
      if (runProcessor)
      {
        thereYet = true;
        runProcessor = false;
      }
      else
      {
        thereYet = false;
      }
    }

    if(isBallOnBoard == false)
    {
      thereYet = true;
      ballsOnBoard++;
    }
    else if (stopTime <= time - startTime)
    {
      thereYet = true;
    }

    SmartDashboard.putBoolean("Auto TY", thereYet); //MAYBE THE ISSUE IS HERE <_---0----------------------

    return thereYet;

  }
}