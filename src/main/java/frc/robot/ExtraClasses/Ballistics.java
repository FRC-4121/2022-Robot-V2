// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ExtraClasses;

/*
 * A class designed to calculate the optimal speed and angle for firing a relatively light missile 
 * to a set height from a given distance, using a single-flywheel mechanism.
 * 
 * Originally designed for FRC #4121 Viking Robotics, 2020 Season
 * 
 * @author: Jonas Muhlenkamp
 */

import static java.lang.Math.*;

import java.util.Scanner;

//Elia File out
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
// Elia


public class Ballistics {


    //Input values
    public double destinationHeight;
    public double launcherHeight;
    public double heightTolerance;
    public double maxSpeedRPM;
    public double wheelDiameter;
    public double wheelSlip;

    //Constants
    public final double g = 386.04;//acceleration due to gravity, in/s^2

    //Math-based values
    public double targetHeight; //change in heights. target height - launcher height.
    public double wheelCircumference;
    public double missileMaxSpeed;
    public double [] timeInAir; //quadratic equations have 2 roots. One of these is a positive time.

    //Ballistics array configuration values
    public static int minDistance = 24;//in inches
    public static int maxDistance = 480;//in inches
    public int distanceIncrement = 1;//not recommended to change this value
    
    //Can be configured for a 'continuous' angle or a two-angle system
    public int minAngle = 50;//in degrees
    public int maxAngle = 50;//in degrees
    public int angleIncrement = 1;//make sure this is a factor of the difference between min and max angle, unless you are only using one angle, then make it 1


    public int minSpeed = 20;//percent
    public int maxSpeed = 100;//percent
    public int argumentCount = 5;//distance, angle, speed, height, possibility(is the shot possible) (0 is false)

    /* 2-d ballistics array
     * Structure:  each internal array is [distance, angle, speed, height, possibility of shot]
     */ 
    public double[][] ballisticsTable;
    //3-14-22 changed this variable to not be static^

    /*
     * Class constructor
     * @param targetH: The height, in inches, of the desired target above the ground
     * @param launchH: The height, in inches, of the launcher release point above the ground
     * @param tolerance: A additive factor, in inches, that widens the range to target around the target height 
     * @param maxRPM: The maximum speed, in rotations per minute, of the flywheel
     * @param wheelD: The diameter, in inches, of the flywheel
     * @param slip: The percent of wheel speed that is converted to linear speed of the missile (due to the single-wheel design, much is lost as rotational speed)
     */
    public Ballistics(double targetH, double launchH, double tolerance, double maxRPM, double wheelD, double slip){

        //Initiailize class values
        destinationHeight = targetH;
        launcherHeight = launchH;
        heightTolerance = tolerance;
        maxSpeedRPM = maxRPM;
        wheelDiameter = wheelD;
        wheelSlip = slip;

        //Calculate needed height from shot
        targetHeight = destinationHeight - launcherHeight;

        //Calculate missile max speed
        wheelCircumference = wheelDiameter * Math.PI;
        missileMaxSpeed = maxSpeedRPM * wheelCircumference * wheelSlip / 60;//in/s //the 60 is to convert RPM to RPS. wheel circumference converts RPS to in/s

        System.out.println("Target Height: " + targetHeight + ", Max Speed: " + missileMaxSpeed);

        //Init ballistics table
        ballisticsTable = generateBallisticsTable();

    }

    public double[][] generateBallisticsTable(){

        //Table to be filled and returned
        double[][] table = new double[(maxDistance - minDistance)/distanceIncrement + 1][argumentCount]; //this makes a matrix of [distance, angle, speed, height, possibility(is the shot possible)] for each of the potential distance

        //Start iterating at hardcoded minimum distance
        double startDistance = minDistance;
      
        //Loop through the distances foot by foot
        for(int i = 0; i < table.length; i++){ //length is number of rows. which is different distances
 
            //At each foot, calculate the optimal angle and speed
            double optimalAngle = 0;
            double optimalSpeed = 0;
            double shotPossible = 0;

            double minError = heightTolerance;
            double bestHeight = 0;
            
            //For each angle...
            for(int a = minAngle; a <= maxAngle; a += angleIncrement){

                //and each speed at each angle...
                for(int s = minSpeed; s <= maxSpeed; s++){

                    //calculate the height that the ball will be at when it hits the wall and error from target
                    double speed = s / 100.0;
                    double height = calculateHeight(startDistance, a, speed); //height of where the shot will land
                    double error = abs(targetHeight - height);

                    //If the error is the smallest yet, assign values of this configuration to place in the table
                    if(error < minError){

                        optimalSpeed = speed;
                        optimalAngle = a;
                        minError = error;
                        bestHeight = height;
                        shotPossible = 1;
                    }

                }

            }

            //Assign table values
            table[i][0] = startDistance;
            table[i][1] = optimalAngle;
            table[i][2] = optimalSpeed;
            table[i][3] = bestHeight;
            table[i][4] = shotPossible;

            //Move to next distance
            startDistance += distanceIncrement;
        }

        return table;
    }

    //Input distance in feet, angle in degrees, speed in percent; output height(the ball will land at distance away) in inches
    public double calculateHeight(double distance, double angle, double speed){
        //this equation looks like the one from @Link:https://byjus.com/trajectory-formula/#:~:text=Trajectory%20formula%20is%20given%20by%20y%20%3D%20xtan%CE%B8%E2%88%92,2%20%CE%B8%20Where%2C%20y%20is%20the%20horizontal%20component%2C
        
        //Derived from parametric equations of t based on basic trajectories.  Does not account for air resistance; this should be accounted for in the 'slip factor'
        double height = tan(toRadians(angle)) * distance - 0.5 * g * pow(distance / (cos(toRadians(angle)) * missileMaxSpeed * speed), 2);
        //The equation^ is
         // y = vt -g/2t^2
         //where t = x(Distance)/vx(horizontal distance )
        return height;
    }
    
    //range formula -- Elia, so it's wrong and doesn't work
    //calculateHorizontalDistanceToGoal
    public double calcHorizDistToGoal( double DegreesAngle )
    {
        missileMaxSpeed = maxSpeedRPM * wheelCircumference * wheelSlip / 60;//in/s //the 60 is to convert RPM to RPS. wheel circumference converts RPS to in/s
        // y = vt -g/2t^2
        //-g/2t^2 + vt - y = 0
        //[quadratic] t=-v +/- sqrt(v^2 -4 * -g/2 * -y) )/2*-4.9
        //v(of y) = v sin angle
        double horizVelocity = missileMaxSpeed * Math.sin(Math.toRadians(DegreesAngle));
        timeInAir[0] = (0-horizVelocity) + Math.sqrt(Math.pow(horizVelocity, 2) - (4* ((0-g)/2) * (0-heightTolerance)));
        //timeInAir[1] = (0-horizVelocity) - Math.sqrt(Math.pow(horizVelocity, 2) - (4* ((0-g)/2) * (0-targetHeight)));
        //System.out.println("HAHAHA timeInAir[0]" + timeInAir[0] + "\n timeInAir[1]" + timeInAir[1]);
        return horizVelocity;
    }

 
    //Actually grab shot configurations from the table
    public double[] queryBallisticsTable(double distance){

        double[] tableValues = new double[4];

        //Default values
        double shotPossible = 0;//essentially a boolean that I can put into an array of doubles; 0 = false, 1 = true
        double angle = 0;
        double speed = 0;
        double dist = 0;
        double distError = 100;//obscenely large to avoid problems

        //If outside the limits of the mechanism, the shot is not possible and we return immediately
        if(distance < (double) minDistance || distance > (double) maxDistance){

            tableValues[0] = 0;
            tableValues[1] = 0;
            tableValues[2] = 0;
            tableValues[3] = 0;

            return tableValues;
        
        //Otherwise, compare the entered distance to the incremented distances in the table and grab data
        } else {

            for(int i = 0; i < ballisticsTable.length; i++){

                double error = abs(ballisticsTable[i][0] - distance);
                //System.out.println(error);
                //System.out.println(distError);

                if(error < distError){ //checks to get best values.

                    distError = error;
                    shotPossible = ballisticsTable[i][4];
                    angle = ballisticsTable[i][1];
                    speed = ballisticsTable[i][2];
                    dist = ballisticsTable[i][0];
                }
            }

            tableValues[0] = shotPossible;
            tableValues[1] = angle;
            tableValues[2] = speed;// * maxSpeedRPM;//performing calculations with percent but controlling via RPM
            tableValues[3] = dist;

            return tableValues;

        }
    }


    public static void main(String[] args){

        

        Ballistics ballistics = new Ballistics(41, 34, 5, 6100, 6, .25);

        //elia
        try {
            
            double[] [] table = ballistics.generateBallisticsTable(); 


            //String address = "C:/Users/113281/Documents/GitHub/2022-Robot-V2/src/main/java/frc/robot/ExtraClasses" + "/outputTable.txt";//you can replace the address...also replace all \ with /
            String address = "C:/Users/team4/OneDrive/Documents/GitHub/2022-Robot-V2/src/main/java/frc/robot/ExtraClasses" + "/outputTable.txt";//you can replace the address...also replace all \ with /
            File file = new File(address);

            // if file doesnt exists, then create it
            if (!file.exists()) {
                file.createNewFile();
            }

            FileWriter fw = new FileWriter(file.getAbsoluteFile());
            BufferedWriter bw = new BufferedWriter(fw);
            bw.write("[distance\t angle\t speed\t height\t possibility] \n");
            for (int i = 0; i< table.length; i++ )
            {
                for(int j = 0; j<5;j++)
                bw.write(" \t " + Math.round(table[i][j]) + " ");
                bw.write("\n");
            }
            bw.close();

            System.out.println("Done");

        } catch (IOException e) {
            e.printStackTrace();
        }
        //elia





        System.out.println("-----------------------------------------");
        System.out.print("4121 Ballistics Program\n");
                    
        Scanner scIn = new Scanner(System.in);

        System.out.print("Start Calculations? (Y/N) ");

        while(scIn.next().toUpperCase().equals("Y")){

            System.out.print("Please enter a distance in inches: ");
            double distance = scIn.nextDouble();

            System.out.println("Attempting shot at distance of " + distance + " inches.");

            double[] tableQuery = ballistics.queryBallisticsTable(distance);
            System.out.println("Calculating...");
            
            if(tableQuery[0] == 1){

                System.out.println("Shot possible at distance of " + distance + " inches.");
                System.out.println("Parameters: ");
                System.out.println("    Angle: " + tableQuery[1] + " degrees.");
                System.out.println("    Speed: " + tableQuery[2] + " percent.");
                System.out.println("    Distance used: " + tableQuery[3] + " inches.");
                //System.out.println("    [this is inaccurate information cuz didn't work ->]Horizontal distance to goal " + ballistics.calcHorizDistToGoal( 60/*degrees*/) +  " inches.");
            } else {

                System.out.println("Shot not possible at distance of " + distance + " inches.");
            }

            System.out.print("\nContinue Calculations? (Y/N) ");

        }

        scIn.close();
    }

}