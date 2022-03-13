// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraController extends SubsystemBase {

  private UsbCamera intakeCamera;
 // private UsbCamera reverseCamera;
  private CameraServer camServer;
  private VideoSink switcher;

  private boolean forwardCamera = true;
  
  public CameraController() {


    
    camServer = CameraServer.getInstance();

    intakeCamera = new UsbCamera("Intake View", 1);
    intakeCamera.setResolution(160, 120);
    intakeCamera.setFPS(15);
    intakeCamera.setBrightness(50);
/*
    reverseCamera = new UsbCamera("Reverse Drive View", 0);
    reverseCamera.setResolution(160, 120);
    reverseCamera.setFPS(15);
    reverseCamera.setBrightness(50);
*/
    switcher = camServer.addSwitchedCamera("Switched");


    intakeCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
   // reverseCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

    switcher.setSource(intakeCamera);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
/*
  public void switchCamera(){

    forwardCamera = !forwardCamera;
    if (forwardCamera)
    {
      switcher.setSource(intakeCamera);
    }
    else
    {
      switcher.setSource(reverseCamera);
    }
  */}
