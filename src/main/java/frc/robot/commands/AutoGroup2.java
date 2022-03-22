// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Processor;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGroup2 extends SequentialCommandGroup {
  /** Creates a new AutoGroup2. */
  public AutoGroup2(Drivetrain drivetrain, Processor processor, Intake intake, NetworkTableQuerier tables, double distance, double stopTime, double delayTime) {
    // Add your commands in the addCommands() call, e.g.
    
    addCommands(new AutoIntakeAndShoot(intake, processor, stopTime, delayTime), new AutoPickUpBall(drivetrain, processor, intake, tables, stopTime));
  }
}
