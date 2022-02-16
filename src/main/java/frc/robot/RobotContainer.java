
package frc.robot;

import static frc.robot.Constants.*;
////import static frc.robot.Constants.ShooterConstants.*;
import frc.robot.subsystems.*;
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





  //===COMMANDS===//

  //Driving Commands
  private final DriveWithJoysticks driveCommand = new DriveWithJoysticks(drivetrain, xbox);

  //Climbing Commands
  private final ExtendClimber extendClimberCommand = new ExtendClimber(climber);
  private final RetractClimber retractClimberCommand = new RetractClimber(climber);


  //Shooting Commands
  private final ShootBall shooterCommand = new ShootBall(shooter);

  //KillAuto Command
  private final KillAutoCommand killAutoObject = new KillAutoCommand(); 

  // Intake
  private final PickUpBall intakeCommand = new PickUpBall(intake, processor);


  //loader
  private final RunLoader runloader = new RunLoader(processor);





  //===BUTTONS===// //They're being initialized in RobotContainer


  //xboxButtons
  private final JoystickButton intakeButton;
  private final JoystickButton climberExtendButton;
  private final JoystickButton climberRetractButton;
  private final JoystickButton shooterButton;
  private final JoystickButton loaderButton;

  
  //launchpad buttons/switches
  private final JoystickButton killAutoButton;
  private final JoystickButton AutoPos1;
  private final JoystickButton AutoPos2;
  private final JoystickButton AutoPos3;
  private static JoystickButton redButton;
  private static JoystickButton blueButton;

  //Driving
  private final JoystickButton invertDirectionButton;
  

  //testing
  private boolean testing = false; //true for xbox, false for launchpad

  //===CONSTRUCTOR===//
  public RobotContainer() {
    
  //colorButtons
  redButton = new JoystickButton(launchpad,LaunchPadSwitch5bottom);
  blueButton = new JoystickButton(launchpad,LaunchPadSwitch5top);

  if(testing) //using xbox controller to test
  {
    //xboxButtons
    intakeButton = new JoystickButton(xbox, xboxAButton);
    climberExtendButton = new JoystickButton(xbox, xboxLeftBumber);
    climberRetractButton = new JoystickButton(xbox, xboxRightBumber);
    shooterButton = new JoystickButton(xbox, xboxAButton);
    loaderButton = new JoystickButton(xbox, xboxBButton);

    //Driving
     invertDirectionButton = new JoystickButton(xbox, 6);
     
  }
  else{ //using launchpad and xbox as if it's a real match
    
     //Command buttons/switches
     intakeButton = new JoystickButton(launchpad, LaunchPadSwitch2bottom);
     climberExtendButton = new JoystickButton(launchpad, LaunchPadSwitch2bottom);
     climberRetractButton = new JoystickButton(launchpad, LaunchPadSwitch2bottom); //get Id's from constants
     shooterButton = new JoystickButton(launchpad, LaunchPadSwitch7);
     loaderButton = new JoystickButton(launchpad, LaunchPadSwitch7);
     

    //Driving
     invertDirectionButton = new JoystickButton(xbox, 6);
     
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
    
    //intake
    intakeButton.whileHeld(intakeCommand);

    //climber
    climberExtendButton.whileHeld(extendClimberCommand);
    climberRetractButton.whileHeld(retractClimberCommand);

    //shooter
    shooterButton.whileHeld(shooterCommand);

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
    return new ExtendClimber(climber);
    // return new AutoGetAllBalls(drivetrain, pneumatics, process2, ntables, ballData, 2, 100);
    // return new AutoShootTimed(drivetrain, shooter, pneumatics, process2, turret, ntables, 60);
    // return new RunHoodToPos(turret, 240);
  }


 
}