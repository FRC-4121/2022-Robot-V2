// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.RotateClimber;


public class ExtendClimber extends CommandBase {

  private final Climber m_climber;
  private double leftSpeed;
  private double rightSpeed;
  private double leftStopPos;
  private double rightStopPos;
  
  //constructor
  public ExtendClimber(Climber climb) {

    // Initialize class variables
    m_climber = climb;
    

    // Add subsystem requirements
    addRequirements(climb);
    
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    leftSpeed = climberSpeed;
    rightSpeed = climberSpeed;
    leftStopPos = 10000000;
    rightStopPos = 10000000;

    if(!climberEncoderInit)
    {
      m_climber.zeroClimberEncoders();
      //r_climber.zeroRotateClimberEncoders();
      climberEncoderInit = true;
    }
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftClimberPos = m_climber.getLeftClimbEncoderPosition();
    if (leftClimberPos < leftStopPos) {
      m_climber.runLeftClimber(leftSpeed);
    }

    double rightClimberPos = m_climber.getRightCLimbEncoderPosition();
    if (rightClimberPos < rightStopPos) {
      m_climber.runRightClimber(-rightSpeed * ClimberExtendLimiter);
    }


  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){

    m_climber.climbStop();

  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean doneYet = false;

     // Only run left climber if within bounds
     double leftClimberPos = m_climber.getLeftClimbEncoderPosition();
     if(leftClimberPos <= climbMinEncoder-1  || leftClimberPos > leftClimbMaxEncoder - climbEncoderTolerance){
      leftSpeed = 0;
      leftStopPos = leftClimberPos;
     }
 
     // Only run right climber if within bounds
     double rightClimberPos = m_climber.getRightCLimbEncoderPosition();
     if(rightClimberPos <= climbMinEncoder-1 || rightClimberPos > rightClimbMaxEncoder - climbEncoderTolerance){
      rightSpeed = 0;
      rightStopPos = rightClimberPos;
     }

     SmartDashboard.putNumber("Left climb encoder", leftClimberPos);
     SmartDashboard.putNumber("Right climb encoder", rightClimberPos);
 
     if(leftSpeed == 0 && rightSpeed == 0)
     {
       doneYet = true;
     }

    return doneYet;

  }
  
}
