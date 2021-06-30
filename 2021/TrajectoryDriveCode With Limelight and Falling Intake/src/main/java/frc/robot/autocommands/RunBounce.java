package frc.robot.autocommands;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
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
import frc.robot.subsystems.DriveTrain;

public class RunBounce extends SequentialCommandGroup {
    Trajectory trajectory;
    Trajectory bouncePath1;
    Trajectory bouncePath2;
    Trajectory bouncePath3;
    Trajectory bouncePath4;

    public RunBounce(DriveTrain drive) {
        
        String myPathName = "";
        String trajectoryfile = "";

        myPathName = "bounce";

        for (int step = 1; step <5; step++) {
            System.out.println("step:");
            System.out.println(step);

            switch (step) {
                case 1:
                    myPathName = "bounce1"; break;
                case 2:
                    myPathName = "bounce2"; break;
                case 3:
                    myPathName = "bounce3"; break;
                case 4:
                    myPathName = "bounce4"; break;
                default: 
                    myPathName = "Invalid myPathName in BouncePath"; break;
            }

            trajectoryfile = myPathName + ".wpilib.json";
            Trajectory trajectory = new Trajectory();
            try {
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryfile);
                trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            } catch (IOException ex) {
                DriverStation.reportError("Unable to open Trajecotry" + trajectoryfile, ex.getStackTrace());
            }

            try {
                trajectoryfile = myPathName + ".txt";
                Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryfile);
                FileWriter fileWriter = new FileWriter(trajectoryPath.toString());
                PrintWriter printWriter = new PrintWriter(fileWriter);
                printWriter.print(trajectory.toString());
                printWriter.close();
            } catch (IOException ex) {
                DriverStation.reportError("Unable to write traj text: " + trajectoryfile, ex.getStackTrace());
            }

            switch (step) {
                case 1:
                    bouncePath1 = trajectory; break;
                case 2:
                    bouncePath2 = trajectory; break;
                case 3:
                    bouncePath3 = trajectory; break;
                case 4:
                    bouncePath4 = trajectory; break;
                default: 
                    System.out.println("invalid option in BouncePaths");
            }
            System.out.println("have path for " + myPathName);
        }

        drive.resetOdometry(bouncePath1.getInitialPose());

        RamseteCommand ramseteCommand1 = new RamseteCommand(
            bouncePath1,
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
            drive::tankDriveVolts,
            drive
        );

        RamseteCommand ramseteCommand2 = new RamseteCommand(
            bouncePath2,
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
            drive::tankDriveVolts,
            drive
        );

        RamseteCommand ramseteCommand3 = new RamseteCommand(
            bouncePath3,
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
            drive::tankDriveVolts,
            drive
        );

        RamseteCommand ramseteCommand4 = new RamseteCommand(
            bouncePath4,
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
            drive::tankDriveVolts,
            drive
        );

        addCommands(
            ramseteCommand1,
            ramseteCommand2,
            ramseteCommand3,
            ramseteCommand4.andThen(() -> drive.tankDriveVolts(0,0))
        );
    }
}
