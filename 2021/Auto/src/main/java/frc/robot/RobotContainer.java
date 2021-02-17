package frc.robot;

import java.util.Arrays;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
 
    private Drivetrain drive = new Drivetrain();

    public Command getAutonomousCommand() {
        TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(2), Units.feetToMeters(2)); //(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq)
        config.setKinematics(drive.getKinematics()); //give Trajectories the Kinematics data

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(Arrays.asList(new Pose2d(), new Pose2d(1.0, 0, new Rotation2d())), config); //Starts at (0,0) Goes to (1,0) ((Moves 1 meter)) and then giving the config
    }

    RamseteCommand command = new RamseteCommand(trajectory, drive::getPose, new RamseteController(2.0, 0.7), drive.getFeedForward(), drive.getKinematics(), drive.getSpeeds(), drive.getLeftPIDController(), drive.getRightPIDController(), drive::setOutput, drive);

    return command.andThen(() -> drive.setOutputVolts(0,0));

    public void reset() {
        drive.reset();
    }
}