// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.*;


public class RaiseIntake extends CommandBase {

  

  private Intake intake = new Intake();
  private Timer timer = new Timer();
  private double startTime;
  private double stopTime;

  /** Creates a new DropIntake. */
  public RaiseIntake(Intake i) {
    intake = i;
    // stopTime = stoptime;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
    startTime = timer.get();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(intake.getIntakeReleaseEncoderPosition() >= intakeRaiseEncoderLimit){
      intake.intakeRaise();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    intake.intakeReleaseStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    double time = timer.get();
    boolean thereYet = false;

    if (intake.getIntakeReleaseEncoderPosition() >= intakeRaiseEncoderLimit ) {
      thereYet = true;
    }
    else if (time - startTime >= stopTime) {
        thereYet = true;
    }

    return thereYet;
  }
  
}