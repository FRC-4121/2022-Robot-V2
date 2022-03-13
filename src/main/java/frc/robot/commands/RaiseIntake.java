// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import static frc.robot.Constants.*;


public class RaiseIntake extends CommandBase {

  private Intake intake = new Intake();

  /** Creates a new RaiseIntake. */
  public RaiseIntake(Intake i) {

    // Initialize class variables
    intake = i;

    // Add subsystem requirements
    addRequirements(intake);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(!intakeEncodersInit)
    {
      intake.zeroIntakeEncoder();
      intakeEncodersInit = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Only move intake if within bounds
    double intakePos = intake.getIntakeReleaseEncoderPosition();
    if (intakePos >= intakeLowerEncoderLimit + intakeEncoderTolerance && intakePos <= intakeRaiseEncoderLimit - intakeEncoderTolerance)
    {
      intake.runIntakeRelease(-kIntakeSpeed);
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

    return false;

  }
  
}