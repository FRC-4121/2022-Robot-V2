// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.ExtraClasses.Ballistics;
import frc.robot.ExtraClasses.PIDControl;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.ExtraClasses.LidarSensor;
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.*;

public class ControlShooterSpeed extends CommandBase {
  
  private Shooter shooter;
  private NetworkTableQuerier ntQuerier;
  private Ballistics ballisticsHigh;
  private Ballistics ballisticsLow;
  private LidarSensor lidar;
  private DigitalInput input;
  private PIDControl pidShooterSpeed;
  private double cameraDistance;
  private double lidarDistance;
  private boolean targetLock;
  private double targetOffset;
  private double targetSpeed;
  private double targetRPM;
  private double currentRPM;
  private double targetShooterSpeedCorrected;
  private double speed;
  private double shooterSpeedCorrection;
  private double shotPossible;//Ballistics value; 0 is false, 1 is true
  private boolean runShooter = false;

  private boolean runSpeedControl = true;

  private double[] ballisticsData;
  
  public ControlShooterSpeed(Shooter shoot, NetworkTableQuerier querier) {

    shooter = shoot;
    ntQuerier = querier;
    ballisticsHigh = new Ballistics(103     , 34, 5, 6050, 6, .25);
    ballisticsLow = new Ballistics(41, 34, 5, 6050, 6, 0.25);
    lidar = new LidarSensor(new DigitalInput(LIDAR_PORT));

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    pidShooterSpeed = new PIDControl(kP_Shoot, kI_Shoot, kD_Shoot);

    speed = .75;
    shooterSpeedCorrection = 0;

    targetSpeed = shooterTargetRPM;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (runShooter) {

      // Get lidar distance
      lidarDistance = lidar.getDistance();
      SmartDashboard.putNumber("Lidar Distance: ", lidarDistance);

      // Get target lock from shooter camera
      targetLock = ntQuerier.getTargetLockFlag();
      SmartDashboard.putBoolean("TargetLock", targetLock);

      if (targetLock) {

        // get the camera distance
        cameraDistance = ntQuerier.getTapeDistance();
        SmartDashboard.putNumber("Camera Distance: ", cameraDistance);

        // ballisticsData = ballistics.queryBallisticsTable(distance);

        shotPossible = ballisticsData[0];

        if (shotPossible == 0) {
          SmartDashboard.putBoolean("Shot Possible", false);
          targetSpeed = speed;
        } else {
          SmartDashboard.putBoolean("Shot Possible", true);
          targetSpeed = ballisticsData[2];
        }
        SmartDashboard.putNumber("BallisticsTarget", targetSpeed);
        targetRPM = targetSpeed * shooterTargetRPM;
        SmartDashboard.putNumber("BallisticsRPM", targetSpeed * shooterTargetRPM);
        currentRPM = Math.abs(shooter.getRPM());

        if (Math.abs(currentRPM - targetRPM) > 50) {
          if (currentRPM < targetRPM) {
            shooterSpeedCorrection += 0.0005;
          } else {
            shooterSpeedCorrection -= 0.0005;
          }
        }

        // Calculate speed correction
        // shooterSpeedCorrection = pidShooterSpeed.run(shooter.getShooterRPM(),
        // targetSpeed*kShooterMaxRPM);
        SmartDashboard.putNumber("correction", shooterSpeedCorrection);

        // Correct shooter speed control input
        // targetShooterSpeedCorrected = targetShooterSpeed * kSpeedCorrectionFactor;
        targetShooterSpeedCorrected = targetSpeed + shooterSpeedCorrection;

        // Ensure corrected speed is within bounds
        if (targetShooterSpeedCorrected > 1) {
          targetShooterSpeedCorrected = 1;
        } else if (targetShooterSpeedCorrected < -1) {
          targetShooterSpeedCorrected = -1;
        }

        // Write key values to dashboard
        SmartDashboard.putNumber("Ballistics Speed", targetShooterSpeedCorrected);

        // targetSpeedCorrected = targetSpeed * kSpeedCorrectionFactor;
        // SmartDashboard.putNumber("Ballistics Speed", targetSpeed);

        shooter.shooterRun(targetShooterSpeedCorrected);
        // I have battery concerns about this implementation. If we notice that battery
        // draw during a match is problematic for speed control, we
        // will need to revert to a pid for RPM in some way. This would be sufficiently
        // complicated that it is a low priority, however.
      } else if (lidarDistance > lidarMin && lidarDistance < lidarMax) /* need a max distance as well */ {

      } else {
        shooterSpeedCorrection = 0;
        speed = 0.4;
        shooter.shooterRun(speed);
      }
    }
    else{
      shooterSpeedCorrection = 0;
      speed = -0.30;
      shooter.shooterRun(speed);
    }

    //SmartDashboard.putNumber("Shooter Speed", shooter.getShooterSpeed());
    SmartDashboard.putNumber("Shooter RPM", shooter.getRPM());
 
 
  }



  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void stopAutoSpeed(){


  }
}