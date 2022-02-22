// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ExtraClasses;

/** BallData Class
 * 
 * Holds values related to the initial ball positions
 * and provides calculations for ball-to-ball angles
 * 
 */
public class BallData {

    // Declare arrays
    private double[] ballDistances;
    private double[] ballAngles;
    private double[] ballOffsets;
    private double[] ballToBallAngles;

    /** 
     * Default constructor 
     */
    public BallData(){

        // Create ball data arrays
        ballDistances = new double[3];
        ballAngles = new double[3];
        ballOffsets = new double[3];
        ballToBallAngles = new double[2];

    }

    /** 
     * Set a ball distance 
     * 
     * @param index
     * @param distance
     */
    public void setBallDistance(int index, double distance){
        ballDistances[index] = distance;
    }

    /** 
     * Set a ball angle 
     * 
     * @param index
     * @param angle
     */
    public void setBallAngle(int index, double angle) {
        ballAngles[index] = angle;
    }

    /** 
     * Set a ball offset 
     * 
     * @param index
     * @param offset
     */
    public void setBallOffset(int index, double offset) {
        ballOffsets[index] = offset;
    }

    /** 
     * Get a ball distance 
     * 
     * @param index
     * @return distance in inches
     */
    public double getBallDistance(int index) {
        return ballDistances[index];
    }

    /** 
     * Get a ball angle 
     * 
     * @param index
     * @return angle in degrees
     */
    public double getBallAngle(int index) {
        return ballAngles[index];
    }

    /** 
     * Get a ball offset
     * 
     * @param index
     * @return offset in inches
     */
    public double getBallOffset(int index) {
        return ballOffsets[index];
    }

    /**
     * Get the angle between two balls
     * 
     * @param index
     * @return angle in degrees
     */
    public double getBallToBallAngle(int index) {
        return ballToBallAngles[index];
    }

    /** 
     * Calculates the angles between the balls 
     * (i.e. angle of ball 2 as seen from ball 1)
     * 
     * Side1: side of triangle perpendicular to longitudinal field axis
     * Side2: side of triangle parallel with longitudinal field axis
     * 
     * Clockwise angles (robot reference) are positive
     */
    public void calcInterBallAngles() {

        // Loop over the balls
        for (int i = 0; i < 2; i++) {

            Double side1 = 0.0;
            Double side2 = 0.0;

            // Calculate sides of triangle
            if (ballAngles[i] == 0.0) {

                side1 = ballOffsets[i+1];
                // Double term1 = ballOffsets[i+1] / Math.tan(Math.toRadians(Math.abs(ballAngles[i+1])));
                side2 = Math.abs(ballDistances[i+1] - ballDistances[i]);

            }
            else if (ballAngles[i+1] == 0.0) {

                side1 = ballOffsets[i];
                // Double term2 = ballOffsets[i] / Math.tan(Math.toRadians(Math.abs(ballAngles[i])));
                side2 = Math.abs(ballDistances[i+1] - ballDistances[i]);

            }
            else {

                side1 = Math.abs(ballOffsets[i]) + Math.abs(ballOffsets[i+1]);
                // Double term1 = ballOffsets[i+1] / Math.tan(Math.toRadians(Math.abs(ballAngles[i+1])));
                // Double term2 = ballOffsets[i] / Math.tan(Math.toRadians(Math.abs(ballAngles[i])));
                // side2 = term1 - term2;
                side2 = Math.abs(ballDistances[i+1] - ballDistances[i]);

            }

            double angle = Math.toDegrees(Math.atan(side1 / side2));

            //Correction for when we need a negative angle.
            if (ballOffsets[i] < ballOffsets[i+1]){
                angle *= -1;
            }
            
            //Compensate for robot overshoot on sharp angles
            // if (angle < -45){
            //     angle = angle - 15;
            // } else if (angle > 45) {
            //     angle = angle + 15;
            // }

            ballToBallAngles[i] = angle;
            // if (i == 0){
            //     ballToBallAngles[i] = 30;
            // } else {
            //     ballToBallAngles[i] = -90;
            // }

        }

    }
}