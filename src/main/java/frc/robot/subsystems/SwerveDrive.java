// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;


public class SwerveDrive extends SubsystemBase {
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {}

  //takes in the current controller input and calculates the new velocity and angle for all of the modules. 
  public void Drive(double leftX, double leftY, double rightX)
  {

    double rate = Math.sqrt((lengthFromAxle * lengthFromAxle) + (widthFromAxle * widthFromAxle));
    leftY *= -1;

    double a = leftX - rightX * (lengthFromAxle / rate);
    double b = leftX + rightX * (lengthFromAxle / rate);
    double c = leftY - rightX * (widthFromAxle / rate);
    double d = leftY + rightX * (widthFromAxle / rate);    

    double backRightSpeed = Math.sqrt ((a * a) + (d * d));
    double backLeftSpeed = Math.sqrt ((a * a) + (c * c));
    double frontRightSpeed = Math.sqrt ((b * b) + (d * d));
    double frontLeftSpeed = Math.sqrt ((b * b) + (c * c));    

    double backRightAngle = Math.atan2 (a, d) / Math.PI;
    double backLeftAngle = Math.atan2 (a, c) / Math.PI;
    double frontRightAngle = Math.atan2 (b, d) / Math.PI;
    double frontLeftAngle = Math.atan2 (b, c) / Math.PI;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
