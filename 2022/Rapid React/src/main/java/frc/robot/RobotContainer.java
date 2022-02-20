// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;

import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.Elevator;
import frc.robot.commands.ElevatorNeutral;
import frc.robot.commands.ElevatorReverse;
import frc.robot.commands.Intake;
import frc.robot.commands.IntakeDrop;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.LedRainbow;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurretAutoAlign;
import frc.robot.commands.TurretCenter;
import frc.robot.commands.TurretNeutral;
import frc.robot.commands.TurretTurnLeft;
import frc.robot.commands.TurretTurnRight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.IntakeSub;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.TurretSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public DriveSubsystem driveSub = new DriveSubsystem();
  

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
  //Initializing Controllers
  public static XboxController xbox = new XboxController(OIConstants.XBOX_ID);
  public static XboxController helms = new XboxController(OIConstants.HELMS_ID);
  public IntakeSub intakeSub = new IntakeSub(); 
  public ElevatorSub elevatorsub = new ElevatorSub();
  public LedSubsystem ledsub = new LedSubsystem();
  public static TurretSubsystem turretSubsystem = new TurretSubsystem();
  public static DriveSubsystem driveSubsystem = new DriveSubsystem();
  public static LedSubsystem ledSubsytem = new LedSubsystem();
  public static ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    driveSub.setDefaultCommand(new ArcadeDrive (driveSub));
  }


  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //turret stuff
    new JoystickButton(helms, Button.kB.value).whenHeld(new TurretAutoAlign(turretSubsystem));
    new JoystickButton(helms, Button.kLeftBumper.value).whenHeld(new TurretTurnLeft(turretSubsystem));
    new JoystickButton(helms, Button.kRightBumper.value).whenHeld(new TurretTurnRight(turretSubsystem));
    new JoystickButton(helms, Button.kA.value).whenHeld(new TurretCenter());
    //neutralizes turret
    new JoystickButton(helms, Button.kB.value).whenReleased(new TurretNeutral(turretSubsystem));
    new JoystickButton(helms, Button.kLeftBumper.value).whenReleased(new TurretNeutral(turretSubsystem));
    new JoystickButton(helms, Button.kRightBumper.value).whenReleased(new TurretNeutral(turretSubsystem));

    
    //intake settings
    new JoystickButton(xbox, Button.kX.value).whenHeld(new Intake(intakeSub));
    new JoystickButton(xbox, Button.kY.value).whenHeld(new IntakeReverse(intakeSub));
    //intake stop
    new JoystickButton(xbox, Button.kX.value).whenReleased(new IntakeStop());
    new JoystickButton(xbox, Button.kY.value).whenReleased(new IntakeStop());

    //intake dropdown
    new JoystickButton(helms, Button.kX.value).whenHeld(new IntakeDrop());
   
    
    //Elevator settings
    new JoystickButton(helms, Button.kA.value).whenHeld(new Elevator(elevatorsub));
    new JoystickButton(helms, Button.kY.value).whenHeld(new ElevatorReverse(elevatorsub));
    //elevator stop
    new JoystickButton(helms, Button.kA.value).whenReleased(new ElevatorNeutral(elevatorsub));
    new JoystickButton(helms, Button.kY.value).whenReleased(new ElevatorNeutral(elevatorsub));

    //shooterstuff
    new JoystickButton(xbox, Button.kB.value).whenHeld(new Shoot());



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null; //Todo ask mikey why annie did this?
  }
}
