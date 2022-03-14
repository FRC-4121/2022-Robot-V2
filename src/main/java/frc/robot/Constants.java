/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

    public static final int plan = 1; //1-4; 1 being leftmost starting position and 4 being right most on the field
    
    //Talon SRX and FX IDs (must be unique, may range from 0+)
    //drivetrain motor IDs
    public static final int LEFT_MASTER_F = 3;
    public static final int LEFT_SLAVE_F = 2;
    public static final int RIGHT_MASTER_F = 1;
    public static final int RIGHT_SLAVE_F = 4;

    //climber motor IDs
    public static final int RIGHT_CLIMBER = 7;
    public static final int LEFT_CLIMBER = 5;
    public static final int RIGHT_ROTATE_CLIMBER = 9; 
    public static final int LEFT_ROTATE_CLIMBER = 11;

    //climber angle encoders
    public static final int LEFT_CLIMBER_ANGLE = 0;
    public static final int RIGHT_CLIMBER_ANGLE = 1;
    
    //processor motor IDs
    public static final int INTAKE=20;
    public static final int INTAKERELEASE = 0;
    public static final int LOADER = 27; 
    public static final int SHOOTER = 10;
    public static final int LEFT_PROCESSOR = 8;
    public static final int RIGHT_PROCESSOR = 6;

    //Drive control port IDs
    public static final int XBOX_PORT = 0;

    //Xbox controller button IDS
    public static final int xboxAButton = 1;
    public static final int xboxBButton = 2;
    public static final int xboxXButton = 3;
    public static final int xboxYButton = 4;
    public static final int xboxLeftBumber = 5;
    public static final int xboxRightBumber = 6;
    public static final int xboxBackButton = 7;//this button is in the middle of the xbox controller
    public static final int xboxStartButton = 8;//this button is in the middle of the xbox controller
    public static final int xboxLeftJoystickButton = 9;
    public static final int xboxRightJoystickButton = 10;

    //LaunchPad button IDs
    public static final int LaunchPadButton1= 7 ;
    public static final int LaunchPadButton2=  17; 
    public static final int LaunchPadButton3= 19 ;
    public static final int LaunchPadButton4=  18;
    public static final int LaunchPadSwitch1bottom=1  ;
    public static final int LaunchPadSwitch1top=2  ;
    public static final int LaunchPadSwitch2bottom=3  ;
    public static final int LaunchPadSwitch2top=4  ;
    public static final int LaunchPadSwitch3 = 5;
    public static final int LaunchPadSwitch4 = 6;
    public static final int LaunchPadSwitch5bottom=8  ;
    public static final int LaunchPadSwitch5top=9  ;
    public static final int LaunchPadSwitch6bottom=10  ;
    public static final int LaunchPadSwitch6top=11 ;
    public static final int LaunchPadSwitch7 = 12;
    public static final int LaunchPadSwitch8 = 13;
    public static final int LaunchPadDial1 = 14; // low bit
    public static final int LaunchPadDial2 = 15;
    public static final int LaunchPadDial3 = 16; // high bit

    //Sensor port IDs
    public static final int LIDAR_PORT = 0;

    //Climber variables
    public static final int kPIDLoopIdxClimb = 0;
    public static final int kTimeoutMsClimb = 20;
    public static final int rightClimbMaxEncoder = 171100; // TBD encoder raw sensor units (2048 in one rotation) for max height that climber should go
    public static final int leftClimbMaxEncoder = 171100;
    public static final int climbMinEncoder = 0;//need to find
    public static final int climbRotateMaxEncoder = -1000000;
    public static final int climbRotateMinEncoder = 1000000;
    public static final int climbEncoderTolerance = 3000;
    public static final double climberSpeed = 0.3;
    public static final double rotateSpeed = 0.5;
    public static double ClimberExtendLimiter = 0.785;
    public static double ClimberRetractLimiter = 0.9;
    public static double rotateClimberLimiter = 0.9;
    public static boolean climberEncoderInit = false;
    public static final double climberStartPos = 50;

    //Shooter variables
    public static double shooterTargetRPM = 100;// need to test to figure out
    public static boolean shootLow = true;
    public static double lidarMin = 3;
    public static double lidarMax = 140;//need to test to find
    public static double visionDistanceTolerance = 10;
    public static double defaultShooterSpeed = 0.3;
    public static final double kP_Shoot = 0.000075; //was 0.1
    public static final double kI_Shoot = 0.0000;
    public static final double kD_Shoot = 0;
    public static final double kF_Shoot = -1;
    public static final double distanceCorrection = 0;//need to find
    public static final int kPIDLoopIdxShoot = 0;
    public static final int kTimeoutMsShoot = 20;
    public static final int kShooterMaxRPM = 6100;
    public static boolean toggleShooterOnOrOff = true; //true runs the shooter motors, false keeps them off.
    
    //Intake variables
    public static final double kIntakeSpeed = 0.1;
    public static int intakeRaiseEncoderLimit = 100000; //need to find out
    public static int intakeLowerEncoderLimit = -100000; //also need to find
    public static int intakeEncoderTolerance = 10;
    public static boolean intakeEncodersInit = false;

    //General variables
    public static boolean killAuto = false;
    public static int ballsOnBoard = 0;
    

    public static class DrivetrainConstants {

        public static final boolean kMotorInvert = true;//True -> right side motors are inverted
        public static final int kPIDLoopIdxDrive = 0;
        public static final int kTimeoutMsDrive = 20;
        public static final double kTalonFXPPR = 2048;
        public static final double kWheelDiameter = 6.0;
        public static final double kLowGearSpeedCap = 0.8;//In case full speed draws excessive power, these are an emergency measure
        public static final double kHighGearSpeedCap = 1.0;
        public static final double kJoystickSpeedCorr = 0.72;
        public static final double kManualDriveSpeed = 0.75;
        public static final double kAutoDriveSpeed = .6;
        public static final double kAutoDriveSpeedMin = 0.25;
        public static final double kAutoShootDriveSpeed = 0.75;
        public static final double kAutoTurnSpeed = 0.5;
        // public static final double kLowGearRatio = 30.0;
        // public static final double kHighGearRatio = 70.0;
        public static final double kGearRatio = 7; 
        public static final double kTurnAngleTolerance = 0.001;
        public static final double kDriveDistanceTolerance = 10.0;
        public static final double AUTO_ENCODER_REVOLUTION_FACTOR = 14750.0;

        public static final double kP_Straight = 0.012;  //was 0.024
        public static final double kI_Straight = 0.0;
        public static final double kD_Straight = 0.0;
        public static final double kP_Turn = .002;//was .003
        public static final double kI_Turn = 0.0;
        public static final double kD_Turn = 0.001;//was 0.0004
        public static final double kP_DriveAngle = .003;//was .005
        public static final double kI_DriveAngle = 0.0;
        public static final double kD_DriveAngle = 0.0004;

        public static final double kSpeedCorrection = 0.9; //this will be used to compensate for differnces in the drive motors

        //Filtering (for gyro)
        public static final int FILTER_WINDOW_SIZE = 150;

        public static int DIRECTION_MULTIPLIER = 1;//Controls whether forward on joysticks is forward or backward on robot
        
        public static double kLowGearMultiplier = 0.40;
        public static double kHighGearMultiplier = 0.80;
        public static double currentGear = kHighGearMultiplier; 
    
    }
}