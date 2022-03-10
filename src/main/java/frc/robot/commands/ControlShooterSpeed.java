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
import static frc.robot.Constants.DrivetrainConstants.*;

import java.nio.file.FileSystemAlreadyExistsException;

import edu.wpi.first.math.filter.MedianFilter;


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
  private double targetDistance;
  private boolean lidarDistanceGood;
  private boolean targetLock;
  private double targetOffset;
  private double targetSpeed;
  private double targetRPM;
  private double currentRPM;
  private double targetShooterSpeedCorrected;
  private double speed;
  private double shooterSpeedCorrection;
  private double shotPossible;//Ballistics value; 0 is false, 1 is true
  private MedianFilter lidarFilter;
  private MedianFilter cameraFilter;
  private double avgLidarDistance;
  private double avgCameraDistance;
  
  private boolean runAutoSpeedControl = true;

  private double[] ballisticsDataHigh;
  private double[] ballisticsDataLow;
  
  public ControlShooterSpeed(Shooter shoot, NetworkTableQuerier querier) {

    // Initialize class variables
    shooter = shoot;
    ntQuerier = querier;

    // Create ballistics tables for high and low goals
    ballisticsHigh = new Ballistics(105, 34, 5, kShooterMaxRPM, 6, .25);
    ballisticsLow = new Ballistics(41, 34, 5, kShooterMaxRPM, 6, 0.25);

    // Initialize LIDAR distance sensor
    lidar = new LidarSensor(new DigitalInput(LIDAR_PORT));

    // Initialize distance filters
    lidarFilter = new MedianFilter(FILTER_WINDOW_SIZE);
    cameraFilter = new MedianFilter(FILTER_WINDOW_SIZE);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shoot);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    pidShooterSpeed = new PIDControl(kP_Shoot, kI_Shoot, kD_Shoot);
    lidarDistanceGood = false;
    shooterSpeedCorrection = 0;
    targetDistance = -1;
    targetSpeed = shooterTargetRPM;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 
    if(toggleShooterOnOrOff){
      
      targetDistance = -1;

      // Check if auto speed control should be run
      if (runAutoSpeedControl)
      {

        // Get lidar distance
        lidarDistance = lidar.getDistance();
        avgLidarDistance = lidarFilter.calculate(lidarDistance);
        SmartDashboard.putNumber("Lidar Distance: ", avgLidarDistance);
        
        //check the distance from lidar
        if(avgLidarDistance >= lidarMin && avgLidarDistance <= lidarMax)
        {
          lidarDistanceGood = true;
        }

        // Get target lock from shooter camera
        targetLock = ntQuerier.getTargetLockFlag();
        SmartDashboard.putBoolean("TargetLock", targetLock);

        // Get camera distance
        cameraDistance = ntQuerier.getTapeDistance();
        avgCameraDistance = cameraFilter.calculate(cameraDistance);
        SmartDashboard.putNumber("Camera Distance: ", avgCameraDistance);

        // Choose proper target distance
        if (false)
        {

          targetDistance = avgCameraDistance;
          SmartDashboard.putNumber("Target Distance: ", targetDistance);

        } else if (lidarDistanceGood) /* need a max distance as well */ {

          targetDistance = avgLidarDistance;
          SmartDashboard.putNumber("Target Distance: ", targetDistance);

        } 

        

        if (targetDistance > 0) {

          // get tballistics data
          if (shootLow) {
            ballisticsDataLow = ballisticsLow.queryBallisticsTable(targetDistance -distanceCorrection);
            shotPossible = ballisticsDataLow[0];
          } else {
            ballisticsDataHigh = ballisticsHigh.queryBallisticsTable(targetDistance-distanceCorrection);
            shotPossible = ballisticsDataHigh[0];
          }
          

          if (shotPossible == 0) {
            SmartDashboard.putBoolean("Shot Possible", false);
            //setting the target speed to the default speed
            targetSpeed = defaultShooterSpeed;
          } else {
            SmartDashboard.putBoolean("Shot Possible", true);
            if (shootLow)
            {
              targetSpeed = ballisticsDataLow[2];
              //System.out.println("LOW" + targetSpeed);
              //System.out.println("Possiblity low" + ballisticsDataLow[0] + "\n and high " + ballisticsDataHigh[0] + " other things idk" + ballisticsDataLow[1] + ballisticsDataLow[2]);
              System.out.println("HELLO" + ballisticsDataHigh[0]);
            } else {
              targetSpeed = ballisticsDataHigh[2];
              //System.out.println("HIGH" + targetSpeed);
            }
            
          }
          SmartDashboard.putNumber("BallisticsTarget", targetSpeed);
          targetRPM = targetSpeed * kShooterMaxRPM;
          SmartDashboard.putNumber("BallisticsRPM", targetSpeed * kShooterMaxRPM);
          currentRPM = Math.abs(shooter.getRPM());

          // Correct speed based on PID
          shooterSpeedCorrection = -pidShooterSpeed.run(currentRPM, targetRPM);

          // if (Math.abs(currentRPM - targetRPM) > 50) {
          //   if (currentRPM < targetRPM) {
          //     shooterSpeedCorrection += 0.0005;
          //   } else {
          //     shooterSpeedCorrection -= 0.0005;
          //   }
          // }

          // Calculate speed correction
          // shooterSpeedCorrection = pidShooterSpeed.run(shooter.getShooterRPM(),
          // targetSpeed*kShooterMaxRPM);
          SmartDashboard.putNumber("Shooter Speed Correction", shooterSpeedCorrection);

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

          shooter.shooterRun(-targetShooterSpeedCorrected);
          // I have battery concerns about this implementation. If we notice that battery
          // draw during a match is problematic for speed control, we
          // will need to revert to a pid for RPM in some way. This would be sufficiently
          // complicated that it is a low priority, however.

        } else {

          //Run shooter at default speed
          shooterSpeedCorrection = 0;
          shooter.shooterRun(-defaultShooterSpeed);

        }

      }
      else
      {

        //Run shooter at default speed
        shooterSpeedCorrection = 0;
        shooter.shooterRun(-defaultShooterSpeed);

      }


      //SmartDashboard.putNumber("Shooter Speed", shooter.getShooterSpeed());
      SmartDashboard.putNumber("Shooter RPM", shooter.getRPM());
    }
    else{
      shooter.shooterStop();
    }
  
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