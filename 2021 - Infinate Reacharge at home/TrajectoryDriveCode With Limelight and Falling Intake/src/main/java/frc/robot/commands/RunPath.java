package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.LimeLightSearch;
import frc.robot.subsystems.DriveTrain;

public class RunPath extends SequentialCommandGroup{

    public RunPath(DriveTrain drive) {
        if (LimeLightSearch.PathToTake = true){
            String trajectoryJSON = "paths/RunGalRedA.wpilib.json";
            Trajectory trajectory = new Trajectory();
            try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }
            
            drive.resetOdometry(trajectory.getInitialPose());

            RamseteCommand command = new RamseteCommand (
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
                
                addCommands(
                    command.andThen(() -> drive.tankDriveVolts(0, 0))
                );
        }
            else {String trajectoryJSON = "paths/RunGalRedB.wpilib.json";
                Trajectory trajectory = new Trajectory();
                try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
                } catch (IOException ex) {
                DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
            }

            drive.resetOdometry(trajectory.getInitialPose());

            RamseteCommand command = new RamseteCommand (
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

        addCommands(
            command.andThen(() -> drive.tankDriveVolts(0, 0))
        );
    }
    }
}
