// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.RotateClimber;


public class RotateClimberBack extends CommandBase {

  public static final int climberMaxRotateEncoder = 0; // Move these to constants
  public static final int climberMinRotateEncoder = 0;
 
  private final RotateClimber m_climber;
  
    /** Creates a new RotateClimberFront. */
    public RotateClimberBack(RotateClimber climber) {
     m_climber = climber;
     
     addRequirements(climber);
      // Use addRequirements() here to declare subsystem dependencies.
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

      if (!climberEncoderInit) {
        m_climber.zeroRotateClimberEncoders();
        climberEncoderInit = true;
      }
    }
  
    // Called every time the scheduler runs while the comma      nd is scheduled.
    @Override
    public void execute() {
      //if(m_climber.getLeftRotateEncoderPosition() >= climberMinRotateEncoder && m_climber.getRightRotateEncoderPosition() >= climberMinRotateEncoder){
        m_climber.rotateLeft(rotateSpeed);
        m_climber.rotateRight(rotateSpeed * rotateClimberLimiter);
      //}
      
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
  
      m_climber.rotateClimbStop();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }