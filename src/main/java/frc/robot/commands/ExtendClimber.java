// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import static frc.robot.Constants.*;


public class ExtendClimber extends CommandBase {

  private final Climber m_climber;
  
  //constructor
  public ExtendClimber(Climber subsystem) {

    m_climber = subsystem;
    addRequirements(subsystem);
    
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_climber.climbExtend(climberSpeed);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){

    m_climber.climbStop();

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean thereYet = false;
    
    if(m_climber.getLeftClimbEncoderPosition() >= climberMaxEncoder || m_climber.getRightCLimbEncoderPosition() >= climberMaxEncoder )
    {
      thereYet = true;
    }
    return thereYet;

  }
  
}
