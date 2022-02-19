// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import static frc.robot.Constants.*;


public class AutoTurnUntilSeeBall extends CommandBase {

  private NetworkTableQuerier ntables;
  private Drivetrain drivetrain = new Drivetrain();
  private double stopTime;
  private double startTime;
  private double ballOffset;
  private double ballDistance;
  private boolean foundBall;
  private Timer timer = new Timer();
  private double angleCorrection, angleError, speedCorrection;
  private boolean holdGyro;
  private boolean ballOnBoard;
  private int direction;
  private double kAutoDriveSpeed = 60;

  /** Creates a new AutoTurnUntilSeeBall. */
  // figure out which way direction takes you. Basically when the rest of the
  // field is on the robot's left, then it would make more sense to rotate there.
  // vice versa.
  public AutoTurnUntilSeeBall(Drivetrain drive, NetworkTableQuerier table, double time, int dir) {

    drivetrain = drive;
    ntables = table;
    stopTime = time;
    direction = dir;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.start();
    startTime = timer.get();

    angleCorrection = 0;
    angleError = 0;
    speedCorrection = 1;
    direction = -1;
    ballOnBoard = true; // what's this variable.

    // WE will only use the gyro in auto so make code in here i think

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get vision values
    ballOffset = ntables.getVisionDouble("BallOffset0");
    ballDistance = ntables.getVisionDouble("BallDistance0");
    foundBall = ntables.getVisionBoolean("FoundBall");
    SmartDashboard.putBoolean("FoundBAll", foundBall);

    // Run drive train
    if (!foundBall) {
      // look for ball by turning. if position
      drivetrain.autoDrive(speedCorrection * direction * kAutoDriveSpeed + angleCorrection * direction,
          speedCorrection * direction * kAutoDriveSpeed - angleCorrection * direction);
    } else {
      drivetrain.autoDrive(kAutoDriveSpeed * direction, direction * kAutoDriveSpeed);// found ball, so move forward.
    }
    SmartDashboard.putNumber("Angle Correction", angleCorrection);
    SmartDashboard.putNumber("direction", direction);
    SmartDashboard.putNumber("Left motor Speed while turning",
        speedCorrection * direction * kAutoDriveSpeed + angleCorrection * direction);
    SmartDashboard.putNumber("Right Motor Speed while turning",
        speedCorrection * direction * kAutoDriveSpeed - angleCorrection * direction);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double time = timer.get();
    if (/* check if the robot intook the ball */ false)
      return true;
    else if (stopTime <= time - startTime)
      return true;
    else if (killAuto == true)
      return true;

    return false;
  }
  
}
