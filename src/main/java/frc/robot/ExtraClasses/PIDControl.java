// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/*
===========================================================================================================================
EXPLINATION
-----------
P.I.D means proportional, integral, derivative control system. 
It is used to control the voltage going into the motor to make it more consistent and fluctuate less.
Error signal: this is the difference between the target and actual values. It’s the amount that needs to be corrected by.
Proportional: is basically using a factor to multiply with the error signal to get correction to input.
Integral: sums the error over time
Derivative: change over time. How fast is error changing over time. This is the constant factor to add to proportional. 
===========================================================================================================================
*/
package frc.robot.ExtraClasses;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDControl {

    private double kP, kI, kD;
    private double targetError, previousError;
    private double errorSum, errorChange;
    private double timeStep = 0.02;
    private double correctionFactor;

    public PIDControl(double gainP, double integralI, double derivativeD){

        kP = gainP;
        kI = integralI;
        kD = derivativeD;

        reset();
    }

    public double run(double sensorReading, double targetValue) {

        correctionFactor = 0;

        targetError = sensorReading - targetValue;

        //Calculate new correction
        errorSum += targetError * timeStep;
        errorChange = (targetError - previousError) / timeStep;
        correctionFactor = kP * targetError + kI * errorSum + kD * errorChange;
        SmartDashboard.putNumber("PContribute", kP * targetError);
        SmartDashboard.putNumber("IContribute", kI * errorSum);
        SmartDashboard.putNumber("DContribute", kD * errorChange);

        //Set previous error
        previousError = targetError;

        return correctionFactor;
    }

    public void reset() {

        errorSum = 0;
        errorChange = 0;
        targetError = 0;
        previousError = 0;
    }
}