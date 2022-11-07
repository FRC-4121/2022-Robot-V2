// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.RotateClimber;
import static frc.robot.Constants.*;


public class RotateClimberFront extends CommandBase {

  public static final int climberMaxRotateEncoder = 0; // Move these to constants
  public static final int climberMinRotateEncoder = 0;
 
private final RotateClimber m_climber;

  /** Creates a new RotateClimberFront. */
  public RotateClimberFront(RotateClimber climber) {
   m_climber = climber;
   
   addRequirements(climber);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(!climberEncoderInit)
    {
      m_climber.zeroRotateClimberEncoders();
      climberEncoderInit = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(m_climber.getLeftRotateEncoderPosition() <= climberMaxRotateEncoder && m_climber.getRightRotateEncoderPosition() <= climberMaxRotateEncoder){
      m_climber.rotateLeft(-rotateSpeed* rotateClimberLimiter);
      m_climber.rotateRight(-rotateSpeed );
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
 