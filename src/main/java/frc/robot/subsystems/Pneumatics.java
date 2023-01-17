// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class Pneumatics extends SubsystemBase {
    
  private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);

  private DoubleSolenoid shifter = new DoubleSolenoid(0, PneumaticsModuleType.CTREPCM,0, 7);
  
  /** Creates a new Pneumatics. */
  public Pneumatics() {
    
    
  
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void extend() {
    
    shifter.set(Value.kReverse);//will require testing of solenoid to confirm
    
  }

  public void retract() {

    shifter.set(Value.kForward);//will require testing of solenoid to confirm
    
  }

}
