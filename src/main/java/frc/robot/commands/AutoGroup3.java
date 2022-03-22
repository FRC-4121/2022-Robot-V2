// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.subsystems.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoGroup3 extends SequentialCommandGroup {
  /** Creates a new AutoGroup3. */
  public AutoGroup3(Drivetrain drivetrain, Processor processor, Intake intake, NetworkTableQuerier tables, double distance, double stopTime, double delayTime) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AutoIntakeAndShoot(intake, processor, stopTime, delayTime), new AutoPickUpBall(drivetrain, processor, intake, tables, stopTime), new AutoDrive(drivetrain, distance, 5, 1, 10), new AutoShoot(processor, 10, 0));
    //new AutoShoot(processor, 10, 2)
    //try 10 deg
    //try shooting w/o goin bak in
    }
}
