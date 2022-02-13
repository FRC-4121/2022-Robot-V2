
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
  private final Joystick launchpad = new Joystick(3);
  

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

  //Processor
  private final RunProcessor processorCommand = new RunProcessor(processor);

  //loader
  private final RunLoader runloader = new RunLoader(processor);





  //===BUTTONS===// //They're being initialized in RobotContainer


  //xboxButtons
  private final JoystickButton intakeButton;
  private final JoystickButton climberExtendButton;
  private final JoystickButton climberRetractButton;
  private final JoystickButton shooterButton;
  private final JoystickButton processorButton;
  
  //launchpad buttons/switches
  private final JoystickButton killAutoButton;
  private final JoystickButton AutoPos1;
  private final JoystickButton AutoPos2;
  private final JoystickButton AutoPos3;

  //Driving
  private final JoystickButton invertDirectionButton;
  private final JoystickButton loaderButton;

  //testing
  private boolean testing = true; //true for xbox, false for launchpad

  //===CONSTRUCTOR===//
  public RobotContainer() {
    
  if(testing) //using xbox controller to test
  {
    //xboxButtons
    intakeButton = new JoystickButton(xbox, xboxAButton);
    climberExtendButton = new JoystickButton(xbox, xboxLeftBumber);
    climberRetractButton = new JoystickButton(xbox, xboxRightBumber);
    shooterButton = new JoystickButton(xbox, xboxAButton);
    processorButton = new JoystickButton(xbox, xboxLeftBumber);
    loaderButton = new JoystickButton(xbox, xboxRightBumber);

    //Driving
     invertDirectionButton = new JoystickButton(xbox, 6);
     
  }
  else{ //using launchpad and xbox as if it's a real match
     intakeButton = new JoystickButton(launchpad, 1);
     climberExtendButton = new JoystickButton(launchpad, 1);
     climberRetractButton = new JoystickButton(launchpad, 1); //get Id's from constants
     shooterButton = new JoystickButton(launchpad, 1);
     processorButton = new JoystickButton(launchpad, 1); 

    //Driving
     invertDirectionButton = new JoystickButton(xbox, 6);
     //loader 
     loaderButton = new JoystickButton(xbox, xboxYButton);

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
    if(testing){ //all xbox buttons
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

    //processor
    processorButton.whileHeld(processorCommand);

    //loader
    loaderButton.whileHeld(runloader);

    }
    else{ //launchpad 
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

    //processor
    processorButton.whileHeld(processorCommand);

    //loader
    loaderButton.whileHeld(runloader);
    }
  }

   

  

  public Command getAutonomousCommand() {
    return new ExtendClimber(climber);
    // return new AutoGetAllBalls(drivetrain, pneumatics, process2, ntables, ballData, 2, 100);
    // return new AutoShootTimed(drivetrain, shooter, pneumatics, process2, turret, ntables, 60);
    // return new RunHoodToPos(turret, 240);
  }


 
}