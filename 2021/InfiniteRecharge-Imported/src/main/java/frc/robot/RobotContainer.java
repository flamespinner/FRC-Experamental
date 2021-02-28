/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
//import edu.wpi.first.wpilibj.Joystick; //TODO ButtonBoardTest
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ArcadeDriveClassic;
import frc.robot.commands.ChangeLimelightAngle;
import frc.robot.commands.ConveyorReverse;
import frc.robot.commands.IndexerBottomBeltRun;
import frc.robot.commands.IntakeBalls;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.ReverseQueue;
import frc.robot.commands.ShootBall;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.QueueSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 * 
 * Contains subsystems, OI devices, and commands (runs commands)
 */
public class RobotContainer {
  // declaring and intializing subsystems(all) and commands (most commands don't need to be delclared/intialized here)
  public DriveSubsystem driveSub      = new DriveSubsystem();
  public QueueSubsystem queueSub     = new QueueSubsystem();
  public ShooterSubsystem shooterSub = new ShooterSubsystem();

  // declaring and intializing controller(s)
  private XboxController xbox = new XboxController(OIConstants.XBOX_ID);
  private XboxController helms = new XboxController(OIConstants.HELMS_ID);
  //private Joystick buttonBoard = new Joystick(OIConstants.buttonBoard); //TODO ButtonBoardTEST

  public static boolean shooting = false;
   
  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //drive controls
    /*driveSub.setDefaultCommand(new ArcadeDriveTrigger(driveSub, 
                              () -> xbox.getTriggerAxis(Hand.kLeft), 
                              () -> xbox.getTriggerAxis(Hand.kRight),
                              () -> xbox.getX(Hand.kLeft)));*/

   driveSub.setDefaultCommand(new ArcadeDriveClassic(driveSub,
                              () -> xbox.getY(Hand.kLeft),
                              () -> xbox.getX(Hand.kRight)*.75));
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

    //Main's Commands - Winch is right stick, 
    //new JoystickButton(xbox, Button.kBumperRight.value).whenHeld(new StartWinch(climberSub)); // Disabled - Climber Taken off
    new JoystickButton(xbox, Button.kA.value).whenHeld(new ShootBall(shooterSub));
    new JoystickButton(xbox, Button.kStart.value).whenHeld(new IntakeReverse(queueSub));
    new JoystickButton(xbox, Button.kX.value).toggleWhenPressed(new ChangeLimelightAngle(shooterSub));

    //Helm's Commands
    //new JoystickButton(helms, Button.kBack.value).whenHeld(new ClimberUp(climberSub)); // Disabled - Climber Taken off
    new JoystickButton(helms, Button.kBumperLeft.value).whenHeld(new IntakeBalls(queueSub)); //TODO might be problem
    new JoystickButton(helms, Button.kB.value).whenHeld(new ConveyorReverse(queueSub));
    new JoystickButton(helms, Button.kX.value).whenHeld(new IndexerBottomBeltRun(queueSub));
    new JoystickButton(helms, Button.kA.value).whenHeld(new ReverseQueue(queueSub));
  }

  public XboxController getController() {
    return xbox; 
  }

  public XboxController getHelms() {
    return helms;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   * 
   * currently there is no auto command
   * 
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    driveSub.zeroHeading();
    driveSub.resetEncoders();
    driveSub.resetOdeometry(new Pose2d(0,0, new Rotation2d(0)));

 /*   String trajectoryJSON = "paths/BarrelRacing.wpilib.json";
Trajectory trajectory = new Trajectory();
try {
  Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
  trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
} catch (IOException ex) {
  DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
}*/
    // Create a voltage constraint to ensure we don't accelerate to fast
    var autoVoltageConstraint =
      new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(AutoConstants.ksVolts,
                                   AutoConstants.kvVoltsSecondsPerMeter,
                                   AutoConstants.kaVoltSecondsSquaredPerMeter),
        driveSub.getKinematics(),
        10);


        // Create config for trajectory
        TrajectoryConfig config =
          new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                               AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(driveSub.getKinematics())
        // Apply the voltage constraint
        .addConstraint(autoVoltageConstraint);

        //Create an example trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          //Start at the origin facing the +X direction
          new Pose2d(0,0, new Rotation2d(0)),
          //Pass through these two interior waypoints, making an 's' curve path
          List.of(
            new Translation2d(1, 1),
            new Translation2d(2, -1)
          ),
          //End 3 meters straight ahead of where we started, facing forward
           new Pose2d(3, 0, new Rotation2d(0)),
           //pass config
           config
        ); 


      RamseteCommand ramseteCommand = new RamseteCommand(
        trajectory, 
        driveSub::getPose, 
        new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta), 
        new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltsSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter), 
        driveSub.getKinematics(), 
        driveSub::getWheelSpeeds, 
        new PIDController(AutoConstants.kPDriveVel, 0, 0),  //Left Controller
        new PIDController(AutoConstants.kPDriveVel, 0, 0),  //Right Controller
        driveSub::tankDriveVolts, 
        driveSub
      );

      /* EXAMPLE RAMSETECOMMAND STRUCTURE
      RamseteCommand = new RamseteCommand(
        trajectory,
        pose,
        controller,
        feedforward,
        kinematics,
        wheelSpeeds,
        leftController,
        rightController,
        outputVolts,
        requirements
      );*/

        //Reset odometry to the starting pose of the trajectory
        driveSub.resetOdeometry(trajectory.getInitialPose());

        //Run path following command, then stop at the end
        return ramseteCommand.andThen(() -> driveSub.tankDriveVolts(0, 0));

  }
}
