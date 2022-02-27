//shooter subsystem -- motors that shoot the ball
//Francesco Canossi

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import static frc.robot.Constants.*;
import com.ctre.phoenix.motorcontrol.ControlMode;



public class Shooter extends SubsystemBase {

  //attributes; variables
  private WPI_TalonFX shooterMotor = new WPI_TalonFX(SHOOTER);
  //private WPI_TalonFX processorMotor = new WPI_TalonFX(27);
  private double speed;

  
  /** Creates a new Shooter. */
  public Shooter(){
  
    //nothing here yet :>

  }


  //methods
  public void shooterStop(){
  
    shooterMotor.set(ControlMode.PercentOutput, 0);
    //processorMotor.set(ControlMode.PercentOutput, 0);
  
  }

  public void shooterRun(){
  
    shooterMotor.set(ControlMode.PercentOutput, -0.36);
    //~30 percent for low goal right against the goal
    //40 decent from one robot length away, 43 without having the shooter warm up to the rpm
    //with 6 feet away, 35 percent or just over 2000 rpm with two balls
    //~50 percent for high goal
    //processorMotor.set(ControlMode.PercentOutput, -0.2);

  } 

  public void shooterRun(double speed){
  
    shooterMotor.set(ControlMode.PercentOutput, speed);
    //~30 percent for low goal
    //~50 percent for high goal
    //processorMotor.set(ControlMode.PercentOutput, -0.2);

  } 
  
  public void shooterRun(int percent){
  
    shooterMotor.set(ControlMode.PercentOutput, percent/100.0);
    //~30 percent for low goal
    //~50 percent for high goal
    //processorMotor.set(ControlMode.PercentOutput, -0.2);

  } 

  public void shootRPM(double rpm){
    //Speed for velocity control is encoder units per 100ms/ so divide by 600 instead of just 60
    speed = rpm * 2048 / 600; // 2048 is the amount of raw sensor units in a rotation
    shooterMotor.set(ControlMode.Velocity, speed);
  }

  public double getRPM()
  {
    return shooterMotor.getSelectedSensorVelocity() * 600 / 2048;
  }

  @Override
  public void periodic() {
  
    // This method will be called once per scheduler run
    
  }
}
