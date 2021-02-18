package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;

public class RobotContainer {
  private Drivetrain drive = new Drivetrain();

  public Command getAutonomousCommand() {
    /*TrajectoryConfig config = new TrajectoryConfig(
        Units.feetToMeters(2.0), Units.feetToMeters(2.0));
    config.setKinematics(drive.getKinematics());

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d()),
            new Pose2d(2.3, 1.2, Rotation2d.fromDegrees(90.0))),
        config
    );*/
    
    String barrelRaceTrajectoryJSON = "paths/BarrelRace.wpilib.json";
    Trajectory trajectory = new Trajectory();
    try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(barrelRaceTrajectoryJSON);
        trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + barrelRaceTrajectoryJSON, ex.getStackTrace());
    }

    RamseteCommand command = new RamseteCommand(
        trajectory,
        drive::getPose,
        new RamseteController(2, .7),
        drive.getFeedForward(),
        drive.getKinematics(),
        drive::getSpeeds,
        drive.getLeftPIDController(),
        drive.getRightPIDController(),
        drive::setOutputVolts,
        drive
    );

    return command.andThen(() -> drive.setOutputVolts(0, 0));
  }

  public void reset() {
    drive.reset();
  }
}
