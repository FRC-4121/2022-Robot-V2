// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Processor;
import static frc.robot.Constants.*;


public class PickUpBall extends CommandBase {

  //creating intake subsystem
  private final Intake mainIntake;
  private final Processor mainProcessor; //processor should run when intake runs
  private boolean isBallOnBoard = true;
  private boolean firstTime = true;

  //constructor
  public PickUpBall(Intake intake, Processor processor){
    mainIntake = intake;
    mainProcessor = processor;
    addRequirements(intake, processor);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
      mainIntake.runIntake();
      mainProcessor.runProcessor(0.1);   
      
      isBallOnBoard = mainProcessor.getIntakeSwitch();
      if(isBallOnBoard == false && firstTime == true) {
        ballsOnBoard++;
        firstTime = false;
      } else if(isBallOnBoard)
      {
        firstTime = true;
      }
      SmartDashboard.putNumber("BallsOnBoard", ballsOnBoard);

  }

  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mainIntake.stopIntake();
    mainProcessor.stopProcessor();
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
  
}
