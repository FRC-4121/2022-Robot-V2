// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.DrivetrainConstants.*;
import frc.robot.ExtraClasses.PIDControl;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Processor;
import frc.robot.subsystems.Drivetrain;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.ExtraClasses.BallData;


/** AutoGetAllBalls Class
 * 
 * This class picks up 3 balls placed in a random pattern on the field
 * 
 */
public class AutoGetAllBalls extends CommandBase {

  // Declare class variables
  private final Drivetrain drivetrain;
  
  private final Processor processor;
  private final NetworkTableQuerier ntables;
  private final BallData balldata;

  private int ballCount;

  private boolean endRun;
  private boolean executeTurn;
  private boolean holdAngle;
  private boolean slowSpeed;
  private boolean markerFound;

  private double ballDistance;
  private double ballOffset;
  private double nextBallAngle;
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


  /** Default constructor */
  public AutoGetAllBalls(Drivetrain drive,  Processor process, NetworkTableQuerier table, BallData data, double deadband, double time) {

    // Set class variables
    drivetrain = drive;
    
    processor = process;
    ntables = table;
    balldata = data;
    angleDeadband = deadband;
    stopTime = time;

    // Add subsystem requirements
    addRequirements(drivetrain, processor);
    
    // Create the PID controller
    pidAngle = new PIDControl(kP_Turn, kI_Turn, kD_Turn);

  }


  /** Initialize this command (only runs first time command is called) */
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
    ballCount = 0;
    targetGyroAngle = 0;
    distanceTraveled = 0;
    totalDistance = 0;
    targetDistance = 150;//in inches; test value, possibly needs slight adjustment
    leftEncoderStart = 0;
    rightEncoderStart = 0;
    leftEncoderStart2 = 0;
    rightEncoderStart2 = 0;

    // Get the initial ball positions and send to SmartDash
    getInitialBallPositions();
    SmartDashboard.putNumber("BallAngle1", balldata.getBallToBallAngle(0));
    SmartDashboard.putNumber("BallAngle2", balldata.getBallToBallAngle(1));

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


  /* Main code to be executed every robot cycle. 
   * The sequence:
   * Phase 1: Drive to the nearest ball, at the appropriate angle
   * -Start at full speed
   * -PID Control to zero the ball offset
   * -Once ball offset is within a certain tolerance, switch PID Control to maintain heading
   * -Drive at full speed
   * -Once ball is within certain distance, slow down and drive at slow speed
   * -Once ball is collected, increment counter and move to next step
   * 
   * DISABLED (all we have to do is set endrun variable false whenever 1 ball comes through) Phase 2: Turn to next ball angle
   * -Grab next angle from BallData
   * -Drive at turn speed
   * -While driving, PID Control to that angle
   * -Once turn is complete revert to full speed and chase ball
   * 
   * DISABLED Phase 3: Collect other balls
   * -Repeat phase 1 and 2 until counter == 3
   * 
   * Phase 4: End run (marker version)
   * -Check if we see markers
   * -While driving at full speed, turn to 0 if no markers, check again
   * -If markers, turn to nearest marker and drive at full speed until user disables
   * 
   * Phase 4: End run (distance version)
   * -Once three balls collected, zero encoders
   * -Orient to 0 and drive at full speed until distance requirement met
   * 
  */
  @Override
  public void execute() {

    // Get gyro angle
    currentGyroAngle = drivetrain.getGyroAngle();

    // Check if executing turn, ending run, or driving to ball
    // These various states will determine the angle and speed corrections for the robot
    if (executeTurn) {

      // Get angle to the next ball
      nextBallAngle = balldata.getBallToBallAngle(ballCount - 1);
      SmartDashboard.putNumber("TargetAngle", nextBallAngle);

      // Calculate angle correction
      angleCorrection = pidAngle.run(currentGyroAngle, nextBallAngle);

      // Determine correct speed correction based on ball angle
      if (Math.abs(nextBallAngle) > 40){
        speedCorrection = 0.7;
      } else {
        speedCorrection = 0.7;
      }

    }
    //If we are finishing the run (because we have three balls)
    else if (endRun){

      //Orient to 0 degrees or nearest marker
      double markerCount = ntables.getVisionDouble("MarkersFound");
      double markerOffset = ntables.getVisionDouble("MarkerOffset0");
      if (markerCount < 1){
        angleCorrection = pidAngle.run(currentGyroAngle, 0);
        speedCorrection = .8;
      } else {
        angleCorrection = pidAngle.run(markerOffset, 0);
        speedCorrection = 1.0;
        markerFound = true;
      }

    } else {//This is normal ball tracking

      // Get current values from vision and gyro
      ballDistance = ntables.getVisionDouble("BallDistance0");
      ballOffset = ntables.getVisionDouble("BallOffset0");



      // Check if we are close enough and centered enough to hold the angle
      if (holdAngle == false) {

        if (ballDistance < 55 && Math.abs(ballOffset) < 4) {

          holdAngle = true;
          targetGyroAngle = currentGyroAngle;

        }
  
      }
    
      // Calculate angle correction for driving
      if (holdAngle == false) {

        angleCorrection = pidAngle.run(ballOffset, 0);

      }
      else {

        angleCorrection = pidAngle.run(currentGyroAngle, targetGyroAngle);

      }
      
      // Determine speed correction based on distance
      if (!slowSpeed)
      {
        if (ballDistance > 60){// && ballCount < 2 && Math.abs(nextBallAngle) < 60 ) {

          speedCorrection = 1;

        } else {

          speedCorrection =.9;
          slowSpeed = true;

        }
      }
      else
      {
        speedCorrection = .9;  
      }
        
    }

    //Send all flags and updating info to SmartDash regardless of command state
    SmartDashboard.putBoolean("ExecuteTurn", executeTurn);
    SmartDashboard.putBoolean("HoldAngle", endRun);
    SmartDashboard.putBoolean("EndRun", endRun);
    SmartDashboard.putNumber("Ball Count", ballCount);
    SmartDashboard.putNumber("Angle Correction", angleCorrection);
    SmartDashboard.putNumber("Speed Correction", speedCorrection);
    SmartDashboard.putNumber("Total Distance", totalDistance);
    SmartDashboard.putNumber("Distance Traveled", distanceTraveled);
    SmartDashboard.putBoolean("MarkerEnd", markerFound);
    
    //Run the processor continually
    processor.runProcessor();

    //Drive based on speed and angle corrections determined within the sequence
    //driving now moved down here, logic tree should just determine the corrections :)
    drivetrain.autoDrive(direction * speedCorrection *  kAutoDriveSpeed + angleCorrection, direction * speedCorrection * kAutoDriveSpeed - angleCorrection);

    //Count encoder revs until such distance that we definitely get past the line
    double totalRotationsRight = Math.abs((drivetrain.getMasterRightEncoderPosition() - rightEncoderStart));
    double totalRotationsLeft = Math.abs((drivetrain.getMasterLeftEncoderPosition() - leftEncoderStart));
    totalDistance = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / AUTO_ENCODER_REVOLUTION_FACTOR;
  
  }


  /** Check to see if command should finish executing */
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

      // Determine if turning or driving
      if (executeTurn) {

        // Get angles
        double currentAngle = drivetrain.getGyroAngle();
        double targetAngle = balldata.getBallToBallAngle(ballCount - 1);
        SmartDashboard.putNumber("TargetAngle", targetAngle);

        // Check for same angle within deadband
        if (Math.abs(targetAngle - currentAngle) <= angleDeadband) {

          executeTurn = false;
          slowSpeed = false;

        }

      //If we are finishing the run
      } else if (endRun) {

        //Count encoder revs until such distance that we definitely get past the line
        double totalRotationsRight = Math.abs((drivetrain.getMasterRightEncoderPosition() - rightEncoderStart2));
        double totalRotationsLeft = Math.abs((drivetrain.getMasterLeftEncoderPosition() - leftEncoderStart2));
        distanceTraveled = (kWheelDiameter * Math.PI * (totalRotationsLeft + totalRotationsRight) / 2.0) / AUTO_ENCODER_REVOLUTION_FACTOR;
      
        // //Check if distance is far enough yet, then stop
        double markerDistance = ntables.getVisionDouble("MarkerDistance0");
        if (markerFound == true && markerDistance <= 31){
          markerFound = false;
          SmartDashboard.putBoolean("MarkerEnd", markerFound);
          thereYet = true;
        }

      } else {

        if(/*drivetrain.getProcessorEntry()  a sensor to detect a ball coming in*/true != false) {

          // Increment ball count
          ballCount++;

          // Check if we have picked up last ball
          if (ballCount == 3) {

            // Move to ending the run
            endRun = true;
            executeTurn = false;

            
            //Reset the encoders for the distance-driving part of the program
            // drivetrain.zeroEncoders();
            leftEncoderStart2 = drivetrain.getMasterLeftEncoderPosition();
            rightEncoderStart2 = drivetrain.getMasterRightEncoderPosition();
            SmartDashboard.putNumber("LeftEncStart2", leftEncoderStart2);
            SmartDashboard.putNumber("RightEncStart2", rightEncoderStart2);
            
            // //Orient to 0 degrees or nearest marker
            // double markerCount = ntables.getVisionDouble("MarkersFound");
            // double markerDistance = ntables.getVisionDouble("MarkerOffset0");
            // if (markerCount >= 1){
            //   targetDistance = markerDistance;
            // }

          } else {

            // Set flags
            executeTurn = true;
            holdAngle = false;

          }
        }
      }

    }

    // Return stopping flag
    return thereYet;
  }


  /** Code to run when this command has finished execution */
  @Override
  public void end(boolean interrupted) {

    //Stop moving things
    drivetrain.stopDrive();
    processor.stopProcessor();
  }


  /**
   *  Gets the initial positions of all three balls
   *  before the robot moves to get them
   */
  private void getInitialBallPositions() {

    // Loop over three balls
    for (int i = 0; i < 3; i++)
    {

      // Distance
      String distanceKey = "BallDistance" + Integer.toString(i);
      Double distance = ntables.getVisionDouble(distanceKey);
      balldata.setBallDistance(i, distance);

      // Angle
      String angleKey = "BallAngle" + Integer.toString(i);
      Double angle = ntables.getVisionDouble(angleKey);
      balldata.setBallAngle(i, angle);

      // Offset
      String offsetKey = "BallOffset" + Integer.toString(i);
      Double offset = ntables.getVisionDouble(offsetKey);
      balldata.setBallOffset(i, offset);

    }

    // Calculate ball to ball angles
    balldata.calcInterBallAngles();
    
  }

}