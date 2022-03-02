
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




public class RobotContainer {
  
  //Driver controllers
  private final XboxController xbox = new XboxController(XBOX_PORT);
  private final Joystick launchpad = new Joystick(0);
  

  //Subsystems
  private final Drivetrain drivetrain = new Drivetrain();
  private final Intake intake = new Intake();
  private final Climber climber = new Climber();
  private final Shooter shooter = new Shooter();
  private final Processor processor = new Processor();


  private final NetworkTableQuerier table = new NetworkTableQuerier();


  //===COMMANDS===//

  //Driving Commands
  private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, xbox);
  private final InvertDirection invertDirectionCommand = new InvertDirection(drivetrain);
  private final ChangeGear changeGearCommand = new ChangeGear(drivetrain);

  //Climbing Commands
  private final ExtendClimber extendClimberCommand = new ExtendClimber(climber);
  private final RetractClimber retractClimberCommand = new RetractClimber(climber);
  private final RotateClimberFront rotateClimberFrontCommand = new RotateClimberFront(climber);
  private final RotateClimberBack rotateClimberBackCommand = new RotateClimberBack(climber);
  //private final AutoClimb autoClimbCommand = new AutoClimb(climber);


  //Shooting Commands
  private final ShootBall shooterCommand = new ShootBall(shooter,processor, table);

  //KillAuto Command
  private final KillAutoCommand killAutoObject = new KillAutoCommand(); 

  // Intake
  private final PickUpBall intakeCommand = new PickUpBall(intake, processor);
  private final DropIntake dropintakeCommand = new DropIntake(intake);
  private final RaiseIntake raiseIntakeCommand = new RaiseIntake(intake);



  //loader
  private final RunLoader runloader = new RunLoader(processor);





  //===BUTTONS===// //They're being initialized in RobotContainer


  //xboxButtons
  private final JoystickButton intakeButton;
  private final JoystickButton moveIntake;
  private final JoystickButton climberExtendButton;
  private final JoystickButton climberRetractButton;
  private final JoystickButton climberRotateFrontButton;
  private final JoystickButton climberRotateBackButton;
  private final JoystickButton shooterButton;
  private final JoystickButton loaderButton;

  
  //launchpad buttons/switches
  private final JoystickButton killAutoButton;
  private final JoystickButton AutoPos1;
  private final JoystickButton AutoPos2;
  private final JoystickButton AutoPos3;
  private static JoystickButton redButton;
  private static JoystickButton blueButton;
  //private final JoystickButton autoClimbButton;

  //Driving
  private final JoystickButton invertDirectionButton;
  private final JoystickButton changeGearButton;
  

  //testing
  private boolean testing = true; //true for xbox, false for launchpad

  //===CONSTRUCTOR===//
  public RobotContainer() {
    
  //colorButtons
  redButton = new JoystickButton(launchpad,LaunchPadSwitch5top);
  blueButton = new JoystickButton(launchpad,LaunchPadSwitch5bottom);

  if(testing) //using xbox controller to test
  {
    //xboxButtons
    moveIntake = new JoystickButton(xbox, xboxBButton);
    intakeButton = new JoystickButton(xbox, xboxLeftBumber); //feeds to processor
    climberExtendButton = new JoystickButton(xbox, xboxStartButton);
    climberRetractButton = new JoystickButton(xbox, xboxBackButton);
    climberRotateFrontButton = new JoystickButton(xbox, xboxXButton);
    climberRotateBackButton = new JoystickButton(xbox, xboxYButton);
    shooterButton = new JoystickButton(xbox, xboxRightBumber);
    loaderButton = new JoystickButton(xbox, xboxAButton);

    //Driving
     invertDirectionButton = new JoystickButton(xbox, 6);
     changeGearButton = new JoystickButton(xbox,7);
     
  }
  else{ //using launchpad and xbox as if it's a real match
    
     //Command buttons/switches
     moveIntake = new JoystickButton(xbox, xboxRightBumber);
     intakeButton = new JoystickButton(launchpad, LaunchPadSwitch2top);
     climberExtendButton = new JoystickButton(launchpad, LaunchPadSwitch2top);
     climberRetractButton = new JoystickButton(launchpad, LaunchPadSwitch2top); //get Id's from constants
     climberRotateFrontButton = new JoystickButton(xbox, xboxXButton);
     climberRotateBackButton = new JoystickButton(xbox, xboxYButton);
     shooterButton = new JoystickButton(launchpad, LaunchPadSwitch7);
     loaderButton = new JoystickButton(launchpad, LaunchPadSwitch7);
     
    //autoClimbButton = new JoystickButton(launchpad, LaunchPadSwitch?);

    //Driving
     invertDirectionButton = new JoystickButton(xbox, 6);
     changeGearButton = new JoystickButton(xbox,7);
     
  }

    //launchpad buttons/switches
    killAutoButton = new JoystickButton(launchpad,LaunchPadButton1);
    AutoPos1 = new JoystickButton(launchpad,LaunchPadDial1);
    AutoPos2 = new JoystickButton(launchpad,LaunchPadDial2);
    AutoPos3 = new JoystickButton(launchpad,LaunchPadDial3);

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
  }
  
  private void configureButtonBindings() {
    
    //drivetrain
    invertDirectionButton.whenPressed(invertDirectionCommand);
    changeGearButton.whenPressed(changeGearCommand);

    //intake
    intakeButton.toggleWhenActive(intakeCommand); //whileHeld
    moveIntake.whileHeld(dropintakeCommand);

    //climber
    climberExtendButton.whileHeld(extendClimberCommand);
    climberRetractButton.whileHeld(retractClimberCommand);
    climberRotateFrontButton.whileHeld(rotateClimberFrontCommand);
    climberRotateBackButton.whileHeld(rotateClimberBackCommand);
    //autoClimbButton.whileHeld(autoClimbCommand);

    //shooter
    shooterButton.toggleWhenActive(shooterCommand);

    //kill auto
    killAutoButton.whenPressed( killAutoObject);
    killAutoButton.whenReleased( killAutoObject);

    //loader
    loaderButton.whileHeld(runloader);
    
    
    
    

    
    
  }

   
  //gets the color selected for the match
  public static int getColorSelection()
  {
    
    if (blueButton.get() == true) {
      return 2;
    } else if (redButton.get() == true) {
      return 1;
    } else {
      return -1;
    }
  }
  

  public Command getAutonomousCommand() {


    //Auto setup
    //plan(in constants): 1-4; 1 being leftmost position and 4 being right most
      if (plan == 1)
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



    //return new ExtendClimber(climber);
    // return new AutoGetAllBalls(drivetrain, pneumatics, process2, ntables, ballData, 2, 100);
    // return new AutoShootTimed(drivetrain, shooter, pneumatics, process2, turret, ntables, 60);
    // return new RunHoodToPos(turret, 240);
  }


 
}