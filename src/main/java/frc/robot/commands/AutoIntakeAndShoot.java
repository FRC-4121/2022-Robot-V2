// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Processor;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntakeAndShoot extends ParallelCommandGroup {
  /** Creates a new AutoIntakeAndShoot. */
  public AutoIntakeAndShoot(Intake intake, Processor processor, double stopTime, double delayTime) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new MoveIntake(intake), new AutoShoot(processor, stopTime, delayTime));
  }
}
