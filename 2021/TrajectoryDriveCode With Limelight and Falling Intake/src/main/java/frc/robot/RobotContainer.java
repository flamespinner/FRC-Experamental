// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autocommands.RunBarrelRace;
import frc.robot.autocommands.RunBounce;
import frc.robot.autocommands.RunGalBlueA;
import frc.robot.autocommands.RunGalBlueB;
import frc.robot.autocommands.RunGalRedA;
import frc.robot.autocommands.RunGalRedB;
import frc.robot.autocommands.RunSlalom;
//import frc.robot.autocommands.RunGalSearch;

import frc.robot.subsystems.DriveTrain;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain drive = new DriveTrain();

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();


  public RobotContainer() {
    configureButtonBindings();
    autoChooser.setDefaultOption("BarrelRace", new RunBarrelRace(drive));
    autoChooser.addOption("Slalom", new RunSlalom(drive));
    autoChooser.addOption("Bounce", new RunBounce(drive));
    //autoChooser.addOption("Galactic Search", new RunGalSearch(drive));
    autoChooser.addOption("GalRedA", new RunGalRedA(drive));
    autoChooser.addOption("GalRedB", new RunGalRedB(drive));
    autoChooser.addOption("GalBlueA", new RunGalBlueA(drive));
    autoChooser.addOption("GalBlueB", new RunGalBlueB(drive));
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}

