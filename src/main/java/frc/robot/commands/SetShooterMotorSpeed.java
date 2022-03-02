// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SetShooterMotorSpeed extends CommandBase {
  private Shooter shooter = new Shooter();
  private Joystick joy;
  public double Speed; //just change it when you need to outside of this class. Also it's in RPM
  /** Creates a new Shoot_Ball. */ 

  public SetShooterMotorSpeed(Shooter S, double speed, Joystick j) {
    shooter = S;
    Speed = speed;
    joy = j;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.shootRPM(Speed);
    shooter.shootRPM(joy.getY() * 2040); //this is definitely wrong but oh well.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
