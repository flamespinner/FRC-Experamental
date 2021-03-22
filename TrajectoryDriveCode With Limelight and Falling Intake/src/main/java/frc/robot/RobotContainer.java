// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.IntakeBalls;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.QueueSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrain drive = new DriveTrain();
  public QueueSubsystem queueSub     = new QueueSubsystem();
  public DriveSubsystem driveSub     = new DriveSubsystem();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    //LOAD A PATH FILE HERE
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    boolean PathToTake;
    if (tv < 1) {
      PathToTake = false;
      
        }
        else {PathToTake = true;}
    
        SmartDashboard.putBoolean("limelight vision", PathToTake);


    if (PathToTake = true){
    String trajectoryJSON = "paths/RedPath.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }
    RamseteCommand command =
    new RamseteCommand(
        trajectory,
        drive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.kS,
            DriveConstants.kV,
            DriveConstants.kA),
        DriveConstants.kDriveKinematics,
        drive::getWheelSpeeds,
        new PIDController(DriveConstants.kP, 0, 0),
        new PIDController(DriveConstants.kP, 0, 0),
        // RamseteCommand passes volts to the callback
        drive::tankDriveVolts,
        drive);

// Reset odometry to the starting pose of the trajectory.
drive.resetOdometry(trajectory.getInitialPose());

// Run path following command, then stop at the end.
return command.andThen(() -> drive.tankDriveVolts(0, 0));
  
  
  }
    else {String trajectoryJSON = "paths/BluePath.wpilib.json";
          Trajectory trajectory = new Trajectory();
          try {
          Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
          trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
          } catch (IOException ex) {
          DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }            RamseteCommand command =
    new RamseteCommand(
        trajectory,
        drive::getPose,
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
        new SimpleMotorFeedforward(
            DriveConstants.kS,
            DriveConstants.kV,
            DriveConstants.kA),
        DriveConstants.kDriveKinematics,
        drive::getWheelSpeeds,
        new PIDController(DriveConstants.kP, 0, 0),
        new PIDController(DriveConstants.kP, 0, 0),
        // RamseteCommand passes volts to the callback
        drive::tankDriveVolts,
        drive);

// Reset odometry to the starting pose of the trajectory.
drive.resetOdometry(trajectory.getInitialPose());

// Run path following command, then stop at the end.

return command.andThen(() -> drive.tankDriveVolts(0, 0));}
    
//END OF LOADING PATH FILE

    // An ExampleCommand will run in autonomous
  
  }
}
