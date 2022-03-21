
package frc.robot;

import static frc.robot.Constants.*;
////import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.*;
import frc.robot.ExtraClasses.NetworkTableQuerier;
import frc.robot.commands.*;
//import frc.robot.extraClasses.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RobotContainer {
  
  //Driver controllers
  private final XboxController xbox = new XboxController(0);
  private final XboxController xboxClimber = new XboxController(1);
  private final Joystick launchpad = new Joystick(2);
  

  //Subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final RotateClimber rotateClimber= new RotateClimber();
  private final Shooter shooter = new Shooter();
  private final Processor processor = new Processor();

  private final NetworkTableQuerier table = new NetworkTableQuerier();


  //===COMMANDS===//

  //Driving Commands
  private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, xbox);

  //Climbing Commands
  private final ExtendClimber extendClimberCommand = new ExtendClimber(climber);
  private final RetractClimber retractClimberCommand = new RetractClimber(climber);
  private final RotateClimberFront rotateClimberFrontCommand = new RotateClimberFront(rotateClimber);
  private final RotateClimberBack rotateClimberBackCommand = new RotateClimberBack(rotateClimber);
  private final RunClimberStart climberStartCommand = new RunClimberStart(climber);
 //private final AutoClimb autoClimbCommand = new AutoClimb(climber);


  //Shooting Commands
  private final ShootBall shooterCommand = new ShootBall(shooter,processor, table);
  private final RunShooter runShooterCommand = new RunShooter(shooter);
  private final ControlShooterSpeed autoShoot = new ControlShooterSpeed(shooter, table);
  private final ToggleShooter toggleShoot = new ToggleShooter();

  //KillAuto Command
  private final KillAutoCommand killAutoObject = new KillAutoCommand(); 

  // Intake
  private final PickUpBall intakeCommand = new PickUpBall(intake, processor);
  private final MoveIntake dropintakeCommand = new MoveIntake(intake);
  private final RaiseIntake raiseIntakeCommand = new RaiseIntake(intake);

  //loader
  private final RunLoader runloader = new RunLoader(processor);

  //launchpad
  private final SetRedBallColor setRedBall = new SetRedBallColor(table);
  private final SetBlueBallColor setBlueBall = new SetBlueBallColor(table);



  //===BUTTONS===// //They're being initialized in RobotContainer


  //xboxButtons
  private final JoystickButton intakeButton;
  private final JoystickButton climberExtendButton;
  private final JoystickButton climberRetractButton;
  private final JoystickButton climberRotateFrontButton;
  private final JoystickButton climberRotateBackButton;
  private final JoystickButton climberStartButton;
  private final JoystickButton shooterButton;
  private final JoystickButton loaderButton;
  private final JoystickButton raiseButton;
  private final JoystickButton runShooterButton;
  private final JoystickButton toggleShooter;
  
  //launchpad buttons/switches
  private final JoystickButton killAutoButton;
  private final JoystickButton AutoPos1;
  private final JoystickButton AutoPos2;
  private final JoystickButton AutoPos3;
  private static JoystickButton redButton;
  private static JoystickButton blueButton;
  private static JoystickButton shooterControlButton;
  private static JoystickButton lowerIntakeButton;
  private static JoystickButton raiseIntakeButton;
  //private final JoystickButton autoClimbButton;

  


  //===CONSTRUCTOR===//
  public RobotContainer() { 
    
  //colorButtons
  redButton = new JoystickButton(launchpad,LaunchPadSwitch5top);
  blueButton = new JoystickButton(launchpad,LaunchPadSwitch5bottom);

  
    //xboxButtons
    intakeButton = new JoystickButton(xbox, xboxLeftBumber); //feeds to processor
    climberExtendButton = new JoystickButton(xboxClimber, xboxRightBumber);
    climberRetractButton = new JoystickButton(xboxClimber, xboxLeftBumber);
    climberRotateFrontButton = new JoystickButton(xboxClimber, xboxYButton);
    climberRotateBackButton = new JoystickButton(xboxClimber, xboxXButton);
    climberStartButton = new JoystickButton(xboxClimber, xboxBButton);
    shooterButton = new JoystickButton(xbox, xboxRightBumber);
    runShooterButton = new JoystickButton(xbox,xboxBButton);
    loaderButton = new JoystickButton(xbox, xboxAButton);
    raiseButton = new JoystickButton(xbox, xboxYButton);
    toggleShooter = new JoystickButton (xboxClimber, xboxAButton);
    //autoClimbButton = new JoystickButton(launchpad, LaunchPadSwitch?);

  

    //launchpad buttons/switches
    killAutoButton = new JoystickButton(launchpad,LaunchPadButton1);
    AutoPos1 = new JoystickButton(launchpad,LaunchPadDial1);
    AutoPos2 = new JoystickButton(launchpad,LaunchPadDial2);
    AutoPos3 = new JoystickButton(launchpad,LaunchPadDial3);
    redButton = new JoystickButton(launchpad, LaunchPadSwitch5top);
    blueButton = new JoystickButton(launchpad, LaunchPadSwitch5bottom);
    shooterControlButton = new JoystickButton(launchpad, LaunchPadSwitch7);
    lowerIntakeButton = new JoystickButton(launchpad, LaunchPadSwitch6bottom);
    raiseIntakeButton = new JoystickButton(launchpad, LaunchPadSwitch6top); 

    //Configure default commands
    configureDefaultCommands();

    // Configure the button bindings
    configureButtonBindings();


  }






  //===METHODS,WHERE STUFF IS CONFIGURED===///


  //For subsystem default commands (driving, etc.)
  private void configureDefaultCommands(){

    //Drivetrain -> drive with xbox joysticks
    drivetrain.setDefaultCommand(driveCommand);

    //Shooter -> run shooter all of the time with auto speed control
    shooter.setDefaultCommand(autoShoot);
  }
  
  private void configureButtonBindings() {
    

    //intake
    intakeButton.whileHeld(intakeCommand); //whileHeld
    lowerIntakeButton.whenPressed(dropintakeCommand);
    raiseIntakeButton.whenPressed(raiseIntakeCommand);

    //climber
    climberExtendButton.whileHeld(extendClimberCommand);
    climberRetractButton.whileHeld(retractClimberCommand);
    climberRotateFrontButton.whileHeld(rotateClimberFrontCommand);
    climberRotateBackButton.whileHeld(rotateClimberBackCommand);
    climberStartButton.whenPressed(climberStartCommand);
    //autoClimbButton.whileHeld(autoClimbCommand);

    //shooter
    shooterButton.whileHeld(shooterCommand);
    toggleShooter.whenPressed(toggleShoot);

    //kill auto
    killAutoButton.whenPressed( killAutoObject);
    killAutoButton.whenReleased( killAutoObject);

    //loader
    loaderButton.whileHeld(runloader);

    //launchpad
    redButton.whenPressed(setRedBall);
    blueButton.whenPressed(setBlueBall);
    
  }

   
  //gets the color selected for the match
  public void getColorSelection()
  {
    
    if (blueButton.get() == true) {
      table.setColor(2);
      SmartDashboard.putString("Ball Color", "Blue");
    } else if (redButton.get() == true) {
      table.setColor(1);
      SmartDashboard.putString("Ball Color", "Red");
    } else {
      table.setColor(1);
      SmartDashboard.putString("Ball Color", "Red");
    }
  }

  public void getShooterSelection()
  {

    if(shooterControlButton.get() == true)
    {
      runAutoSpeedControl = false;
    } else {
      runAutoSpeedControl = true;
    }

  }
  

  public Command getAutonomousCommand() {


    //Auto setup
    //plan(in constants): 1-4; 1 being leftmost position and 4 being right most
   /*   if (plan == 1)
    {
      return new AutoGroup(intake, shooter, drivetrain, table, processor, 0, 12, 1); //change numbers.
    }
    else if (plan == 2)
    {
      return new AutoGroup(intake, shooter, drivetrain, table, processor, 0, 12,  1);
    }
    else if (plan == 3)
    {
      return new AutoGroup(intake, shooter, drivetrain, table, processor, 0, 12,  -1 );
    }
    else if (plan == 4)
    {
      return new AutoGroup(intake, shooter, drivetrain, table, processor, 0, 12,  -1);
    }
    return null;
*/


    //return new AutoGroup1(processor, drivetrain, intake, 65, 4, 10);
    return new AutoGroup2(drivetrain, processor, intake, table, 120, 10, 3);
    //return new AutoPickUpBall(drivetrain, processor, intake, table, 10);
  }


 
}