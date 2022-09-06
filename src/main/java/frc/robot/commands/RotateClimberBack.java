// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.*;
import frc.robot.subsystems.RotateClimber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class RotateClimberBack extends CommandBase {

  public static final int climberMaxRotateEncoder = 0; // Move these to constants
  public static final int climberMinRotateEncoder = 0;
  private double leftSpeed;
  private double rightSpeed;
  private double leftStopPos;
  private double rightStopPos;
  double leftClimberPos;
  double rightClimberPos;

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

      leftSpeed = climberSpeed;
      rightSpeed = climberSpeed;
      leftStopPos = 10000000;
      rightStopPos = 10000000;
    
      if(!climberRotateEncoderInit)//happens only once @ beginning 
      {
        m_climber.zeroRotateClimberEncoders();
        climberRotateEncoderInit = true;
      }
    }
  
    // Called every time the scheduler runs while the comma      nd is scheduled.
    @Override
    public void execute() {


      leftClimberPos = m_climber.getLeftRotateEncoderPosition();
      if (leftClimberPos <= climbRotateMinEncoder - 1 || leftClimberPos > leftRotateMaxEncoder - climbEncoderTolerance) {
        m_climber.rotateLeft(0);
      } else {
        m_climber.rotateLeft(leftSpeed);
      }

      rightClimberPos = m_climber.getRightRotateEncoderPosition();
      if (rightClimberPos <= climbRotateMinEncoder - 1 || rightClimberPos > rightRotateMaxEncoder - climbEncoderTolerance) {
        m_climber.rotateRight(0);
      } else {
        m_climber.rotateRight(rightSpeed * rotateClimberLimiter);
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
        
      boolean doneYet = false;

      // Only run left climber if within bounds
      leftClimberPos = m_climber.getLeftRotateEncoderPosition();
      if(leftClimberPos <= climbRotateMinEncoder-1  || leftClimberPos > leftRotateMaxEncoder - climbEncoderTolerance){
       leftSpeed = 0;
       leftStopPos = leftClimberPos;
      }
  
      // Only run right climber if within bounds
      rightClimberPos = m_climber.getRightRotateEncoderPosition();
      if(rightClimberPos <= climbRotateMinEncoder-1 || rightClimberPos > rightRotateMaxEncoder - climbEncoderTolerance){
       rightSpeed = 0;
       rightStopPos = rightClimberPos;
      }
 
      SmartDashboard.putNumber("Left rotate encoder", leftClimberPos);
      SmartDashboard.putNumber("Right  rotate encoder", rightClimberPos);
  
      if(leftSpeed == 0 && rightSpeed == 0)
      {
        doneYet = true;
      }
 
     return doneYet;
    }
  }