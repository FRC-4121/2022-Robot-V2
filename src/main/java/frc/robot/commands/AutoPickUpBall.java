// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Constants.*;
import frc.robot.ExtraClasses.PIDControl;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Processor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.ExtraClasses.NetworkTableQuerier;


public class AutoPickUpBall extends CommandBase {

  // Declare class variables
  private final Drivetrain drivetrain;
  private final Processor processor;
  private final Intake intake;
  private final NetworkTableQuerier ntables;

  private boolean endRun;
  private boolean executeTurn;
  private boolean holdAngle;
  private boolean slowSpeed;
  private boolean markerFound;

  private double ballDistance;
  private double ballOffset;
  private double direction;
  private double stopTime;
  private double startTime;
  private double angleCorrection;
  private double speedCorrection;
  private double currentGyroAngle;
  private double targetGyroAngle;
  private double angleDeadband;

  private double distanceTraveled;
  private double totalDistance;
  private double targetDistance;
  private double leftEncoderStart;
  private double rightEncoderStart;
  private double leftEncoderStart2;
  private double rightEncoderStart2;

  private Timer timer;

  private PIDControl pidAngle;

  public AutoPickUpBall(Drivetrain drive, Intake in, Processor process, NetworkTableQuerier table, double deadband,
      double time) {

    // Set class variables
    drivetrain = drive;
    processor = process;
    intake = in;
    ntables = table;
    angleDeadband = deadband;
    stopTime = time;

    // Add subsystem requirements
    addRequirements(drivetrain, intake, processor);

    // Create the PID controller
    pidAngle = new PIDControl(kP_Turn, kI_Turn, kD_Turn);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Start the timer and get the command start time
    timer = new Timer();
    timer.start();
    startTime = timer.get();

    // Initialize class variables
    direction = -1;
    angleCorrection = 0;
    speedCorrection = 1;
    targetGyroAngle = 0;
    distanceTraveled = 0;
    totalDistance = 0;
    targetDistance = 150;// in inches; test value, possibly needs slight adjustment
    leftEncoderStart = 0;
    rightEncoderStart = 0;
    leftEncoderStart2 = 0;
    rightEncoderStart2 = 0;

    // Initialize command state control flags
    executeTurn = false;
    endRun = false;

    // Initialize driving control flags
    holdAngle = false;
    slowSpeed = false;
    markerFound = false;

    // Zero gyro angle
    drivetrain.zeroGyro();

    drivetrain.zeroEncoders();
    leftEncoderStart = drivetrain.getMasterLeftEncoderPosition();
    rightEncoderStart = drivetrain.getMasterRightEncoderPosition();
    SmartDashboard.putNumber("LeftEncStart", leftEncoderStart);
    SmartDashboard.putNumber("RightEncStart", rightEncoderStart);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Get current values from vision and gyro
    ballDistance = ntables.getVisionDouble("BallDistance0");
    ballOffset = ntables.getVisionDouble("BallOffset0");

    // Check if we are close enough and centered enough to hold the angle
    if (holdAngle == false) {

      if (ballDistance < 55 && Math.abs(ballOffset) < 4) {

        holdAngle = true;
        targetGyroAngle = currentGyroAngle;

      }

    } else if (holdAngle == false) {

      // Calculate angle correction for driving
      angleCorrection = pidAngle.run(ballOffset, 0);

    } else {

      // Calculate angle correction for driving
      angleCorrection = pidAngle.run(currentGyroAngle, targetGyroAngle);

    } 

    // Determine speed correction based on distance
    if (!slowSpeed) {
      if (ballDistance > 60) {// && ballCount < 2 && Math.abs(nextBallAngle) < 60 ) {

        speedCorrection = 1;

      } else {

        speedCorrection = .9;
        slowSpeed = true;

      }
    } else {
      speedCorrection = .9;
    }

    // Send all flags and updating info to SmartDash regardless of command state
    SmartDashboard.putBoolean("ExecuteTurn", executeTurn);
    SmartDashboard.putBoolean("HoldAngle", endRun);
    SmartDashboard.putBoolean("EndRun", endRun);
    SmartDashboard.putNumber("Ball Count", ballsOnBoard);
    SmartDashboard.putNumber("Angle Correction", angleCorrection);
    SmartDashboard.putNumber("Speed Correction", speedCorrection);
    SmartDashboard.putNumber("Total Distance", totalDistance);
    SmartDashboard.putNumber("Distance Traveled", distanceTraveled);

    // Run the processor and intake
    processor.runProcessor();
    intake.runIntake();

    // Drive based on speed and angle corrections determined within the sequence
    // driving now moved down here, logic tree should just determine the corrections
    // :)
    drivetrain.autoDrive(direction * speedCorrection * kAutoDriveSpeed + angleCorrection,
        direction * speedCorrection * kAutoDriveSpeed - angleCorrection);

    // Count encoder revs until such distance that we definitely get past the line
    double totalRotationsRight = Math.abs((drivetrain.getMasterRightEncoderPosition() - rightEncoderStart));
    double totalRotationsLeft = Math.abs((drivetrain.getMasterLeftEncoderPosition() - leftEncoderStart));
    totalDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0)
        / AUTO_ENCODER_REVOLUTION_FACTOR;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    drivetrain.stopDrive();
    processor.stopProcessor();
    intake.stopIntake();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Initialize stopping flag and send to SmartDash
    boolean thereYet = false;
    SmartDashboard.putBoolean("ThereYet", thereYet);

    // Get current time
    double time = timer.get();

    // Check for max time
    if (stopTime <= time - startTime) {

      // Set flag
      thereYet = true;

    } else {

      // If we are finishing the run
      if (endRun) {

        // Count encoder revs until such distance that we definitely get past the line
        double totalRotationsRight = Math.abs((drivetrain.getMasterRightEncoderPosition() - rightEncoderStart2));
        double totalRotationsLeft = Math.abs((drivetrain.getMasterLeftEncoderPosition() - leftEncoderStart2));
        distanceTraveled = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0)
            / AUTO_ENCODER_REVOLUTION_FACTOR;

        // //Check if distance is far enough yet, then stop
        double markerDistance = ntables.getVisionDouble("MarkerDistance0");
        if (markerFound == true && markerDistance <= 31) {
          markerFound = false;
          SmartDashboard.putBoolean("MarkerEnd", markerFound);
          thereYet = true;
        }

      } // else {

      // if(drivetrain.getProcessorEntry() == false) {

      // Increment ball count
      // ballCount++;

      // Check if we have picked up last ball
      // if (ballsOnBoard == 2) {

      // Move to ending the run
      // endRun = true;
      // executeTurn = false;

      // Reset the encoders for the distance-driving part of the program
      // drivetrain.zeroEncoders();
      // leftEncoderStart2 = drivetrain.getMasterLeftEncoderPosition();
      // rightEncoderStart2 = drivetrain.getMasterRightEncoderPosition();
      // SmartDashboard.putNumber("LeftEncStart2", leftEncoderStart2);
      // SmartDashboard.putNumber("RightEncStart2", rightEncoderStart2);

      // //Orient to 0 degrees or nearest marker
      // double markerCount = ntables.getVisionDouble("MarkersFound");
      // double markerDistance = ntables.getVisionDouble("MarkerOffset0");
      // if (markerCount >= 1){
      // targetDistance = markerDistance;
      // }

      else {

        // Set flags
        executeTurn = true;
        holdAngle = false;

      }
    }
    
    // Return stopping flag
    return thereYet;

  }

}
