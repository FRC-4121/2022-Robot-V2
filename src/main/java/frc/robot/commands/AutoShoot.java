// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Processor;
import static frc.robot.Constants.*;
import edu.wpi.first.wpilibj.Timer;


public class AutoShoot extends CommandBase {

  //attributes; variables
  private final Processor processor;
  private Timer timer = new Timer();
  private double startTime;
  private double endTime;
  private double delayTime;

  
  /** Creates a new ShootBall. */
  public AutoShoot(Processor process, double stoptime, double delay) {

    processor = process;
    endTime = stoptime;
    delayTime = delay;
    addRequirements(processor);

  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    timer.start();
    startTime = timer.get();

  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){

    double currentTime = timer.get();
    double elapsedTime = currentTime - startTime;

    // Load the balls to the shooter once the shooter gets up to speed
    if (elapsedTime >= delayTime && OKToShoot) {

      processor.runLoader(0.2);
      processor.runProcessor(0.15);

    }
    
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){

    processor.stopLoader();
    processor.stopProcessor();

  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    boolean doneYet = false;
    
    if (isBallShot == 1)
    {
      ballsOnBoard--;
    }

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
