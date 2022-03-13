// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.*;

public class AutoMoveIntake extends CommandBase {

    // Declare class variables
    private Intake intake;
    private Timer timer;
    private double intakeSpeed;
    private String intakeDirection;
    private double startTime;
    private double stopTime;
  
  /** Creates a new AutoMoveIntake. */
  public AutoMoveIntake(Intake i, double stoptime) {

    // Initialize class variables
    intake = i;
    stopTime = stoptime;
    intakeSpeed = 0.1;
    intakeDirection = "UP";

    // Add subsystem requirements
    addRequirements(intake);

    // Create a new timer
    timer = new Timer();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // Start the timer and get current time
    timer.start();
    startTime = timer.get();

    // Get the move direction based 
    if(intake.getIntakeReleaseEncoderPosition() <= intakeRaiseEncoderLimit){

      intakeSpeed = 0.1;
      intakeDirection = "DOWN";

    }
    else
    {

      intakeSpeed = -0.1;
      intakeDirection = "UP";

    }     

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Run the intake release motor
    intake.runIntakeRelease(intakeSpeed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // Stop the intake motor
    intake.intakeReleaseStop();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double currentTime = timer.get();
    boolean thereYet = false;

    if (intakeDirection == "UP")
    {

      if (intake.getIntakeReleaseEncoderPosition() >= intakeRaiseEncoderLimit - intakeEncoderTolerance)
      {
        thereYet = true;
      }

    }
    else if (intakeDirection == "DOWN")
    {
      
      if (intake.getIntakeReleaseEncoderPosition() <= intakeLowerEncoderLimit + intakeEncoderTolerance)
      {
        thereYet = true;
      }

    }
    else if (currentTime - startTime >= stopTime)
    {

      thereYet = true;

    }

    return thereYet;

  }
}
