// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//this puts the climber at a starting position... hopefully.
public class RunClimberStart extends CommandBase {
 


  private Climber climber;

  /** Creates a new RunClimberStart. */
  public RunClimberStart(Climber climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    climber = climb;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  
    climber.zeroClimberEncoders();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Only run left climber if within bounds
    double leftClimberPos = climber.getLeftClimbEncoderPosition();
    if(leftClimberPos <= climberStartPos){
      climber.runLeftClimber(climberSpeed);
    }

    // Only run right climber if within bounds
    double rightClimberPos = climber.getRightCLimbEncoderPosition();
    if(rightClimberPos <= climberStartPos){
      climber.runRightClimber(-climberSpeed * ClimberExtendLimiter);
    }

    SmartDashboard.putNumber("Left climb encoder", leftClimberPos);
    SmartDashboard.putNumber("Right climb encoder", rightClimberPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.climbStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean doneYet = false;
    double leftClimberPos = climber.getLeftClimbEncoderPosition();
    double rightClimberPos = climber.getRightCLimbEncoderPosition();
    if(leftClimberPos > climberStartPos && rightClimberPos > climberStartPos)
    {
      doneYet = true;
    }
    
    return doneYet;
  }
}
