// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import static frc.robot.Constants.*;

public class RotateClimberBack extends CommandBase {

  public static final int climberMaxRotateEncoder = 0; // Move these to constants
  public static final int climberMinRotateEncoder = 0;
 
  private final Climber m_climber;
  
    /** Creates a new RotateClimberFront. */
    public RotateClimberBack(Climber climber) {
     m_climber = climber;
     
     addRequirements(climber);
      // Use addRequirements() here to declare subsystem dependencies.
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the comma      nd is scheduled.
    @Override
    public void execute() {
      if(m_climber.getLeftRotateEncoderPosition() >= climberMinRotateEncoder && m_climber.getRightRotateEncoderPosition() >= climberMinRotateEncoder){
        m_climber.rotateClimbIn(rotateSpeed);
      }
      
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