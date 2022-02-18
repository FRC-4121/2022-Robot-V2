// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Intake extends SubsystemBase {

  //motors
  private WPI_TalonSRX intakeMain = new WPI_TalonSRX( INTAKE);
  private WPI_TalonFX intakeRelease = new WPI_TalonFX(INTAKERELEASE);


  /** Creates a new Intake. */
  public Intake() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // method to run the motor for the intake wheels
  public void runIntake()
  {
    intakeMain.set(-0.6);
  }
 

  //method to stop the motor for the intake wheels
  public void stopIntake()
  {
      intakeMain.set(0);
  }


  //method to start the motor for the intake release
  public void intakeRelease()
  {
      intakeRelease.set(0.1);
  }


  //method to stop the motor for the intake release 
  public void intakeReleaseStop()
  {
    //sets motor speed to stop
    intakeRelease.set(0);
  }

  

}
