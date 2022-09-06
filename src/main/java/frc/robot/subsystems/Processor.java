// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.CANSparkMax;

public class Processor extends SubsystemBase {

  //Limit switches to count balls on board
  private DigitalInput intakeSwitch = new DigitalInput(1);
  
  //we have two motors facing each other but both running to feed in, so one of them must be in the opposite direction AKA negative.
  private CANSparkMax leftProcessor = new CANSparkMax(LEFT_PROCESSOR,CANSparkMax.MotorType.kBrushless);
  private CANSparkMax rightProcessor = new CANSparkMax(RIGHT_PROCESSOR,CANSparkMax.MotorType.kBrushless);

  //loader motor
  private CANSparkMax loader = new CANSparkMax(LOADER,CANSparkMax.MotorType.kBrushless);


  /** Creates a new Processor. */
  public Processor() {}


  @Override
  public void periodic() { 
    
    SmartDashboard.putBoolean("Intake Switch", getIntakeSwitch());
    
  }


  // method to run tyhe motor for the processor wheels
  public void runProcessor(double speed)
  {
    leftProcessor.set(speed);
    rightProcessor.set(-speed);
    
  }


  // method to stop the motor for the processor wheels
  public void stopProcessor()
  {
    leftProcessor.set(0);
    rightProcessor.set(0);
    
  }
 

  public void runLoader(double speed)
  {
     loader.set(speed);
  }


  public void stopLoader()
  {
    loader.set(0);
  }  

  public boolean getIntakeSwitch()
  {

    return intakeSwitch.get();

  }

  

}
