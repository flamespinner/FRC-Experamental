// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
import frc.robot.commands.RunIntake;

import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.QueueSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class GalacticSearchRamsete extends CommandBase {
  private Drive_Train _drivetrain;
  private Intake _intake;
  private RamseteCommand _ramsete;

  /** Creates a new GalacticSearchRamsete. */
  public GalacticSearchRamsete(Drive_Train drivetrain, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    _drivetrain = drivetrain;
    _intake = intake;
    addRequirements(_drivetrain, _intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _ramsete = null;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (_ramsete == null) {
      _ramsete = createRamseteCommand();
      _ramsete.initialize();
      return;
    }

    _intake.intakeIn();
    _ramsete.execute();
  }














  private RamseteCommand createRamseteCommand() {

    SmartDashboard.putBoolean("limelight vision", PathToIntake);


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














  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _intake.stop();
    _ramsete.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return _ramsete.isFinished();
  }
}
