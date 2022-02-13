// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


public class Processor extends SubsystemBase {

  //we have two motors facing each other but both running to feed in, so one of them must be in the opposite direction AKA negative.
  private WPI_TalonSRX processor1 = new WPI_TalonSRX(PROCESSOR_1);
  private WPI_TalonSRX processor2 = new WPI_TalonSRX(PROCESSOR_2);

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
    processor1.set(-0.2);
    processor2.set(0.2);
    
  }


  // method to stop the motor for the processor wheels
  public void stopProcessor()
  {
    processor1.set(0);
    processor2.set(0);
    
  }
 

  public void runLoader()
  {
     loader.set(0.2);
  }


  public void stopLoader()
  {
    loader.set(0);
  }  

}
