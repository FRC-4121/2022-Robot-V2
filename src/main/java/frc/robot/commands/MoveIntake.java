// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class MoveIntake extends CommandBase {

  // Declare class variables
  private Intake intake;

  //time
  private double startTime;
  private Timer timer;
  private double stopTime = 2;
  private double time;

  /** Creates a new DropIntake. */
  public MoveIntake(Intake i) {

    // Initialize class variables
    intake = i;
    timer = new Timer();
    // Add subsystem requirements
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

      intake.zeroIntakeEncoder();
      intakeEncodersInit = true;
      timer.start();
      startTime = timer.get();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { 

    // Run the intake release motor
    if (intakePosition != "DOWN");
    {
      intake.runIntakeRelease(kIntakeSpeed);
    }

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
    boolean doneYet = false;

    time = timer.get();

    if (intakePosition == "DOWN")
    {

      doneYet = true;

    }
    else if (time - startTime >= stopTime)
    {
      doneYet = true;
      intakePosition = "DOWN";
    }

    SmartDashboard.putString("intake Pos",intakePosition);

    return doneYet;
  }
  
}
