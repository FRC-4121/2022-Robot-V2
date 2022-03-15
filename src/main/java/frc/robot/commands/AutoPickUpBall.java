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
import static frc.robot.Constants.*;

public class AutoPickUpBall extends CommandBase {
  
  private final Drivetrain drivetrain;
  private final Processor processor;
  private final NetworkTableQuerier ntables;
  // private final Processor processor;

  private double targetDistance;
  private double targetAngle;
  private double direction;
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


  public AutoPickUpBall(Drivetrain drive, Processor process,NetworkTableQuerier table, double time) {

    processor = process;
    drivetrain = drive;
    ntables = table;
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

    holdGyro = false;
    targetGyro = drivetrain.getGyroAngle();
    actualGyro = drivetrain.getGyroAngle();

    isBallOnBoard = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get vision values
    double ballOffset = ntables.getVisionDouble("BallOffset0");
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
    
    SmartDashboard.putNumber("ActualGyro", actualGyro);
    SmartDashboard.putNumber("TargetGyro", targetGyro);

    // Calculate correction based on ball offset if far away
    // or based on gyro angle if close enough
    if (holdGyro == false){
      angleCorrection = pidAngle.run(ballOffset, 0);
    } else {
      angleCorrection = pidAngle.run(actualGyro, targetGyro);
    }
    
    // Calculate speed correction based on distance
    if ((ballDistance > 30) && foundBall == true){
      speedCorrection = 1;
    }else{
      speedCorrection = 1;
    }
    SmartDashboard.putNumber("SpeedCorrect", speedCorrection);

    // Run drive train
    direction = -1;
    if (foundBall) {
      double leftDriveSpeed = (speedCorrection * direction * kAutoDriveSpeed) + angleCorrection;
      SmartDashboard.putNumber("Left Drive Speed", leftDriveSpeed);
      double rightDriveSpeed = (speedCorrection * direction * kAutoDriveSpeed) - angleCorrection;
      SmartDashboard.putNumber("Right Drive Speed", rightDriveSpeed);


      SmartDashboard.putNumber("EAspeedCorrection",speedCorrection);
      SmartDashboard.putNumber("EAdirection",direction);
      SmartDashboard.putNumber("EAkAutoDriveSpeed",kAutoDriveSpeed);
      SmartDashboard.putNumber("EAangleCorrection",angleCorrection);


      drivetrain.autoDrive(leftDriveSpeed, rightDriveSpeed);
    } else {
      drivetrain.stopDrive();//modified for showcase purposes
    }
    SmartDashboard.putNumber("Angle Correction", angleCorrection);
    // processor.runProcessor(false);
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

    /*double time = timer.get();

    isBallOnBoard = processor.getIntakeSwitch();
 
    SmartDashboard.putNumber("Auto Time", time);
    SmartDashboard.putNumber("Auto Start Time", startTime);
    SmartDashboard.putBoolean("BallOnBoard", isBallOnBoard);

    if(isBallOnBoard == true) {
      thereYet = true;
      ballsOnBoard++;
    }
    else if (stopTime <= time - startTime){
      thereYet = true;
    }
    SmartDashboard.putBoolean("Auto TY", thereYet);*/ //MAYBE THE ISSUE IS HERE <_---0----------------------
    return thereYet;

  }
}