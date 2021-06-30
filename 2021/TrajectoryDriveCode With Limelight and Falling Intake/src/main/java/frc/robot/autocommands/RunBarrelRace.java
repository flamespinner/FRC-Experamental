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



public class RunBarrelRace extends SequentialCommandGroup {
    Trajectory trajectory;

    public RunBarrelRace(DriveTrain drive) {

        String myPathName = "";
        String trajectoryfile = "";

        myPathName = "BarrelRace";

        trajectoryfile = myPathName + ".wpilib.json";
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryfile);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryfile, ex.getStackTrace());
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

        drive.resetOdometry(trajectory.getInitialPose());

        RamseteCommand m_ramsetecommand = new RamseteCommand(
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
            drive::tankDriveVolts,
            drive
        );
        
        addCommands(
            m_ramsetecommand.andThen(() -> drive.tankDriveVolts(0,0))
        );
    }
}
