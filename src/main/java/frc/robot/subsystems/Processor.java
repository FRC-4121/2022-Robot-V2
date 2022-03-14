// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;



public class Processor extends SubsystemBase {

  //Limit switches to count balls on board
  private DigitalInput intakeSwitch = new DigitalInput(1);
  


  //we have two motors facing each other but both running to feed in, so one of them must be in the opposite direction AKA negative.
  private WPI_TalonSRX leftProcessor = new WPI_TalonSRX(LEFT_PROCESSOR);
  private WPI_TalonSRX rightProcessor = new WPI_TalonSRX(RIGHT_PROCESSOR);

  //loader motor
  private WPI_TalonSRX loader = new WPI_TalonSRX(LOADER);


  /** Creates a new Processor. */
  public Processor() {}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


  // method to run tyhe motor for the processor wheels
  public void runProcessor()
  {
    leftProcessor.set(0.15);
    rightProcessor.set(-0.15);
    
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
    System.out.println(intakeSwitch.get());
    return intakeSwitch.get();
  }

  

}
