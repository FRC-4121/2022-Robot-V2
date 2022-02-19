// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Timer;


public class AutoClimb extends CommandBase {
  
  Climber climber = new Climber();
  Drivetrain drivetrain = new Drivetrain();
  Timer timer = new Timer();
  double stopTime;
 
  /** Creates a new AutoClimb. */
  //time for stop tima cannot be higher than 30, because the end gfame is only 30 seconds
  public AutoClimb(Climber climb, Drivetrain drive,double time) {
    
    climber = climb;
    drivetrain = drive;
    stopTime = time;
    addRequirements(climb,drive);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //lots needs to go here
    //make sure we are behind the first rung
    //extend up
    //pull down until the second bars get over, go back down until it is supporting the robot
    //rotate until we are at the correct angle to extend the arms to hold on the second rung
    //rotate back forward and then down to rest the arms on the second rung, and pull up to where the second bards are in position like the first
    //repeat the process with new constants to get to the third rung
    
    //if(in position) f
    //climber.climbExtend(0.2);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    climber.rotateClimbStop();
    climber.climbStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean doneYet = false;
   
    if(killAuto = true)
    {
      doneYet = true;
    } 
    if(timer.get() >= stopTime)
    {
      doneYet = true;
    }
    
    return doneYet;
  }
}
