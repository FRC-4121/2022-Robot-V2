// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.DrivetrainConstants.*;
import edu.wpi.first.wpilibj.XboxController;


public class DriveWithJoysticks extends CommandBase {

  private Drivetrain drivetrain;
  private XboxController Xbox;
  private boolean gearChanged;
  private boolean directionChanged;


  public DriveWithJoysticks(Drivetrain drive, XboxController xbox) {

    drivetrain = drive;
    Xbox = xbox;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    gearChanged = false;
    directionChanged = false;
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if((Xbox.getRightTriggerAxis() >= 0.5) && !gearChanged){

      drivetrain.changeGears();
      gearChanged = true;
    } else if(Xbox.getRightTriggerAxis() == 0)
    {
      gearChanged = false;
    }

    if((Xbox.getLeftTriggerAxis() >= 0.5) && !directionChanged){

      drivetrain.invertDirection();
      directionChanged = true;
    } else if(Xbox.getLeftTriggerAxis() == 0)
    {
      directionChanged = false;
    }

    // Drive using xbox joystick values
    // kSpeedCorrection is to slow down the right motors because left motors were
    // running slower
    drivetrain.drive(kJoystickSpeedCorr * Xbox.getLeftY(), kJoystickSpeedCorr * Xbox.getRightY());

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
  
}
