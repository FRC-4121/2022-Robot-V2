// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Processor;


public class RunProcessor extends CommandBase {

  //creating Processor subsystem
  private final Processor processor;
  

  /** Creates a new RunProcessor. */
  public RunProcessor(Processor p){
    // Use addRequirements() here to declare subsystem dependencies.
    processor = p;
    //addRequirements(processor);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    processor.runProcessor();
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    processor.stopProcessor();
  }

  
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
