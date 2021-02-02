/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.OIConstraints;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private Joystick JoystickL = new Joystick(Constants.Joystick_ID_L); 
  //private Joystick JoystickR = new Joystick(Constants.Joystick_ID_R);

  
  //public double leftDriveIntake = JoystickL.getY();
  //public double rightDriveIntake = JoystickR.getY();
  private final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  private XboxController m_XboxController = new XboxController(OIConstraints.Xbox_ID);



  //drive.tankDrive(leftDriveIntake,rightDriveIntake);
 
  

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);
 



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
  /*m_driveSubsystem.setDefaultCommand(new DriveCommand(m_driveSubsystem, 
                              () -> m_XboxController.getY(Hand.kLeft),
                              () -> m_XboxController.getY(Hand.kRight)));*/

  m_driveSubsystem.tankDrive(m_XboxController.getY(Hand.kLeft), m_XboxController.getY(Hand.kRight));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
   // new JoystickButton(xbox, Button.kA.value).whenHeld(new IntakeCommand(m_intakeSubsystem)); 
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
