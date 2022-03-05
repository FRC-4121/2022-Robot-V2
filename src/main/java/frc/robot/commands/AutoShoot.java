// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Processor;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Timer;


public class AutoShoot extends CommandBase {

  //attributes; variables
  private final Shooter shooter;
  private final Processor processor;
  private Timer timer = new Timer();
  private double endTime;
  
  /** Creates a new ShootBall. */
  public AutoShoot(Shooter subsystem, Processor anotherSubsystem, double time) {
    shooter = subsystem;
    processor = anotherSubsystem;
    endTime = time;
    addRequirements(subsystem,anotherSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.start();
    //We are going to need some kind of way to make sure we are targeting the goal, if we do it in here
    //we also need to find the distance from the goal that we are, and change the speed accordingly

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){

    shooter.shooterRun(-30);

    // Load the balls to the shooter once the shooter gets up to speed
    if (Math.abs(shooter.getRPM()) >= shooterTargetRPM) {
      processor.runLoader(0.2);
      processor.runProcessor();
    }
  
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
    boolean doneYet = false;
    
    if(ballsOnBoard == 0)
    {
      doneYet = true;
    }
    if(timer.get() >= endTime)
    {
      doneYet = true;
    }
    if(killAuto == true )
    {
      doneYet = true;
    }

    return doneYet;
  }
  
}