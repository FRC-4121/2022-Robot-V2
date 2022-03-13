// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.RotateClimber;


public class RetractClimber extends CommandBase {

  private final Climber m_climber;

   
  //constructor
  public RetractClimber(Climber climb) {
    
    // Initialize class variables
    m_climber = climb;

    // Add subsystem requirements
    addRequirements(climb);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    if(!climberEncoderInit)
    {
      m_climber.zeroClimberEncoders();
      //m_climber.zeroRotateClimberEncoders();
      climberEncoderInit = true;
    }
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Only run left climber if within bounds
    double leftClimberPos = m_climber.getLeftClimbEncoderPosition();
   // if(leftClimberPos > climbMinEncoder && leftClimberPos <= leftClimbMaxEncoder + 1){
      m_climber.runLeftClimber(-climberSpeed);
   // }

    // Only run right climber if within bounds
    double rightClimberPos = m_climber.getRightCLimbEncoderPosition();
   // if(rightClimberPos > climbMinEncoder && rightClimberPos <= rightClimbMaxEncoder + 1){
      m_climber.runRightClimber(climberSpeed * ClimberRetractLimiter);
   // }
      
    SmartDashboard.putNumber("Left climb encoder", leftClimberPos);
    SmartDashboard.putNumber("Right climb encoder", rightClimberPos);

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){

    // Stop climber motors
    m_climber.climbStop();

  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;

  }
  
}
