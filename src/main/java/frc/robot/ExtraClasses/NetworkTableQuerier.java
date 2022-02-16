// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.ExtraClasses;

import java.lang.Thread;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class NetworkTableQuerier implements Runnable {

    // Create network tables
    private static NetworkTableInstance networkTableInstance;
    private static NetworkTable visionTable;
    private static NetworkTable navxTable;

    // Create network table entries
    private static NetworkTableEntry robotStop;
    private static NetworkTableEntry zeroGyro;
    private static NetworkTableEntry piGyroAngle;
    private static NetworkTableEntry ballDistance0;
    private static NetworkTableEntry ballAngle0;
    private static NetworkTableEntry ballOffset0;
    private static NetworkTableEntry ballScreenPercent0;
    private static NetworkTableEntry ballDistance1;
    private static NetworkTableEntry ballAngle1;
    private static NetworkTableEntry ballOffset1;
    private static NetworkTableEntry ballScreenPercent1;
    private static NetworkTableEntry ballDistance2;
    private static NetworkTableEntry ballAngle2;
    private static NetworkTableEntry ballOffset2;
    private static NetworkTableEntry ballScreenPercent2;
    private static NetworkTableEntry markerDistance0;
    private static NetworkTableEntry markerAngle0;
    private static NetworkTableEntry markerOffset0;
    private static NetworkTableEntry markerDistance1;
    private static NetworkTableEntry markerAngle1;
    private static NetworkTableEntry markerOffset1;
    private static NetworkTableEntry foundBall;
    private static NetworkTableEntry foundTape;
    private static NetworkTableEntry tapeDistance;
    private static NetworkTableEntry tapeOffset;
    private static NetworkTableEntry targetLock;
    private static NetworkTableEntry saveVideo; 
    private static NetworkTableEntry markersFound;
    private static NetworkTableEntry colorSelection;

    // Declare class variables
    private boolean runNetworkTables;


    /**
     * Class constructor
     */
    public NetworkTableQuerier(){

        // Initialize the network tables
        initNetworkTables();

        // Set flags
        runNetworkTables = true; 

    }
    

    /**
     * Main execution thread
     */
    public void run(){
        
        while(runNetworkTables){

            queryNetworkTables();
             
        }
    }


    /**
     * Start the main execution thread
     */
    public void start(){

        runNetworkTables = true;
        Thread ntThread = new Thread(this);
        ntThread.start();

    }


    /**
     * Stop the main execution thread
     */
    public void stop(){

        runNetworkTables = false;
        
    }

    public NetworkTable getVisionTable(){
        return visionTable;
    }

    /**
     * Initialize network tables
     */
    private void initNetworkTables(){

        networkTableInstance = NetworkTableInstance.getDefault();
        visionTable = networkTableInstance.getTable("vision");
        navxTable = networkTableInstance.getTable("navx");

        robotStop = visionTable.getEntry("RobotStop");
        colorSelection = visionTable.getEntry("ColorSelection");
        zeroGyro = navxTable.getEntry("ZeroGyro");

        robotStop.setNumber(0);
        zeroGyro.setNumber(0);

        queryNetworkTables();

    }


    /**
     * Get values from network tables
     */
    private void queryNetworkTables(){

        robotStop = visionTable.getEntry("RobotStop");
        zeroGyro = navxTable.getEntry("ZeroGyro");

        piGyroAngle = navxTable.getEntry("GyroAngle");

        ballDistance0 = visionTable.getEntry("BallDistance0");
        ballAngle0 = visionTable.getEntry("BallAngle0");
        ballOffset0 = visionTable.getEntry("BallOffset0");
        ballScreenPercent0 = visionTable.getEntry("BallScreenPercent0");
        ballDistance1 = visionTable.getEntry("BallDistance1");
        ballAngle1 = visionTable.getEntry("BallAngle1");
        ballOffset1 = visionTable.getEntry("BallOffset1");
        ballScreenPercent1 = visionTable.getEntry("BallScreenPercent1");
        ballDistance2 = visionTable.getEntry("BallDistance2");
        ballAngle2 = visionTable.getEntry("BallAngle2");
        ballOffset2 = visionTable.getEntry("BallOffset2");
        ballScreenPercent2 = visionTable.getEntry("BallScreenPercent2");
        markerDistance0 = visionTable.getEntry("MarkerDistance0");
        markerAngle0 = visionTable.getEntry("MarkerAngle0");
        markerOffset0 = visionTable.getEntry("MarkerOffset0");
        markerDistance1 = visionTable.getEntry("MarkerDistance1");
        markerAngle1 = visionTable.getEntry("MarkerAngle1");
        markerOffset1 = visionTable.getEntry("MarkerOffset1");
        markersFound = visionTable.getEntry("MarkersFound");
        foundBall = visionTable.getEntry("FoundBall");


        foundTape = visionTable.getEntry("FoundTape");
        targetLock = visionTable.getEntry("TargetLock");
        tapeDistance = visionTable.getEntry("TapeDistance");
        tapeOffset = visionTable.getEntry("TapeOffset");
        saveVideo = visionTable.getEntry("SaveVideo");

        SmartDashboard.putBoolean("TargetLock", targetLock.getBoolean(false));
        SmartDashboard.putNumber("TapeOffset", tapeOffset.getDouble(0));
    }

    /*
     * @param entry The ID of the NetworkTables entry to return
     * @return the double value of the NetworkTables entry chosen; an error will be returned if entry is not a double 
     * 
     * List of available entries:
     * "BallDistance"
     * "BallAngle"
     * "BallScreenPercent"
     * "TapeOffset"
     * "TapeDistance" 
     */
    public synchronized double getVisionDouble(String entry){

        return visionTable.getEntry(entry).getDouble(0);
    }
    public synchronized void putVisionDouble(String entry, double value){
        visionTable.getEntry(entry).setDouble(value);
    }

    /*
     * @param entry The ID of the NetworkTables entry to return
     * @return the boolean value of the NetworkTables entry chosen; an error will be returned if entry is not a boolean 
     * 
     * List of available entries:
     * "FoundBall"
     * "FoundTape"
     * "TargetLock" 
     */
    public synchronized boolean getVisionBoolean(String entry){

        return visionTable.getEntry(entry).getBoolean(false);
    }


    /**
     * Get the Found Tape flag
     * @return
     */
    public synchronized boolean getFoundTapeFlag(){

        return foundTape.getBoolean(false);

    }


    /**
     * Get the Target Lock flag
     * @return
     */
    public synchronized boolean getTargetLockFlag(){

        return targetLock.getBoolean(false);

    }


    /**
     * Get the tape offset
     * @return
     */
    public synchronized double getTapeOffset(){

        return tapeOffset.getDouble(0.0);

    }


    /**
     * Get the tape distance
     * @return
     */
    public synchronized double getTapeDistance(){

        return tapeDistance.getDouble(0.0);

    }


    /**
     * Get the VMX gyro angle
     * @return
     */
    public synchronized double getPiGyro(){

        return navxTable.getEntry("GyroAngle").getDouble(0);
    }


    /**
     * Set the robot stop flag
     */
    public synchronized void robotStop(){

        robotStop.setNumber(1);
    }


    /**
     * Zero the VMX gyro
     */
    public synchronized void zeroPiGyro(){

        zeroGyro.setNumber(1);
    }

}