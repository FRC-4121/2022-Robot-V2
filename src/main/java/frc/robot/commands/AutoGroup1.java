// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Processor;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGroup1 extends SequentialCommandGroup {
  /** Creates a new AutoGroup1. */
  public AutoGroup1(Processor processor, Drivetrain drivetrain, Intake intake, double distance, double delaytime, double stoptime) {
    // Add your commands in the addCommands() call, e.g.
    
    addCommands(new MoveIntake(intake), new AutoShoot(processor, stoptime, delaytime), new AutoIntakeAndMoveGroup(intake, drivetrain, distance, stoptime));
  }
}