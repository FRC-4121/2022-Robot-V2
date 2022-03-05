// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Processor;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.ExtraClasses.Ballistics;
import static frc.robot.Constants.*;


public class ShootBall extends CommandBase {

  //attributes; variables
  private final Shooter shooter;
  private final Processor processor;
  private final NetworkTableQuerier table;
  
  
  /** Creates a new ShootBall. */
  public ShootBall(Shooter subsystem, Processor process, NetworkTableQuerier ntables) {
    shooter = subsystem;
    processor = process;
    table = ntables;
    addRequirements(subsystem, process);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    shooter.shooterRun(-60);


  //  if (Math.abs(shooter.getRPM()) >= shooterTargetRPM) {
    processor.runLoader(0.2);
    processor.runProcessor();
   // }
    SmartDashboard.putNumber("Shooter RPM", shooter.getRPM());
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    shooter.shooterStop();
    processor.stopLoader();
    processor.stopProcessor();
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
