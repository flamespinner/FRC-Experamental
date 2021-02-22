/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;



//Auto Imports
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import com.kauailabs.navx.frc.AHRS;

/**
 * this subsystem sets up and directly manipulates everything on the drive train
 */
public class DriveSubsystem extends SubsystemBase {

  //declaring and intializing drive motor controllers and assciated configuration objects
  private final WPI_TalonFX falconBL = new WPI_TalonFX(DriveConstants.FALCON_BL_ID); 
  private final WPI_TalonFX falconFL = new WPI_TalonFX(DriveConstants.FALCON_FL_ID); 
  private final WPI_TalonFX falconBR = new WPI_TalonFX(DriveConstants.FALCON_BR_ID); 
  private final WPI_TalonFX falconFR = new WPI_TalonFX(DriveConstants.FALCON_FR_ID); 

  private final TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  private final SpeedControllerGroup SCG_R = new SpeedControllerGroup(falconFR, falconBR); 
  private final SpeedControllerGroup SCG_L = new SpeedControllerGroup(falconFL, falconBL); 

  private final DifferentialDrive drive = new DifferentialDrive(SCG_L, SCG_R);

  private double error;
  private double integral;

  //define Gyro
  AHRS gyro = new AHRS(SPI.Port.kMXP);

  //DifferentialDrive Kinematics and Odometry
  public static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24)); //TODO check value here
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
    AutoConstants.ksVolts,
    AutoConstants.kvVoltsSecondsPerMeter,
    AutoConstants.kaVoltSecondsSquaredPerMeter
  );
  // kS, kV, Ka
  //kS: Volts
  //kV: Volts * Seconds / Meters
  //kA: Volts * Seconds^2 / Meters
  //"theoretical" test to check kV. 12Volts/TheoreticalFreeSpeedofDriveTrain = ~kV
  //TheoreticalFreeSpeedofDriveTrain = free speed of the motor * wheel circumference / gear reduction


  //Define PIDControllers
  PIDController leftPidController = new PIDController(
    AutoConstants.kPDriveVel,
    0,
    0
  );
  PIDController rightPidController = new PIDController(
    AutoConstants.kPDriveVel,
    0,
    0
  );
  //PIDController(kp, ki, kd)

  //Define 2D Position data x, y
  Pose2d pose; 


  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    setBrake();
    //setting ramp
    falconFR.configOpenloopRamp(1.0); // 0.5 seconds from neutral to full output (during open-loop control)
    falconFR.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)

    falconFL.configOpenloopRamp(1.0); // 0.5 seconds from neutral to full output (during open-loop control)
    falconFL.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)

    falconBL.configOpenloopRamp(1.0); // 0.5 seconds from neutral to full output (during open-loop control)
    falconBL.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)

    falconBR.configOpenloopRamp(1.0); // 0.5 seconds from neutral to full output (during open-loop control)
    falconBR.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)

    //Drive Base Code
    falconBR.follow(falconFR); //talonBR follows TalonFR
    falconBL.follow(falconFL); //talonBL follows TalonFR 

    //falconFR.setInverted(true); //set to invert falconFR.. CW/CCW.. Green = forward (motor led)
    falconBR.setInverted(InvertType.FollowMaster); //matches whatever falconFR is

    //falconFL.setInverted(true); //set to invert falconFL.. CW/CCW.. Green = foward (motor led)
    falconBL.setInverted(InvertType.FollowMaster); //matches whatever falcon FL is
    //Encoder Code Start
    fxConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; //Selecting Feedback Sensor
   
    resetEncoders();
  }

  //DifferntialDrive Speeds and such
  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      falconFR.getSelectedSensorVelocity() / AutoConstants.gearRatio * 2 * Math.PI * Units.inchesToMeters(3.0) / 60, //RPM to meters per second
      falconFL.getSelectedSensorVelocity() / AutoConstants.gearRatio * 2 * Math.PI * Units.inchesToMeters(3.0) / 60
      //Velocity / 10.71(Gear Ratio) = RPM 
      // 2 * PI * 0.0762 (Wheel size in Meters 3" Radius) = Meters Per Minute
      //Using inches to meters to convert from an inch value (3) to meters ( 0.0762)
      //Meters per minute / 60 = Meters per second
    );
  }


  public void setCoast() {
    //setting coast or brake mode, can also be done in Phoenix tuner
    falconFR.setNeutralMode(NeutralMode.Coast);
    falconFL.setNeutralMode(NeutralMode.Coast);
    falconBR.setNeutralMode(NeutralMode.Coast);
    falconBL.setNeutralMode(NeutralMode.Coast);
  }
  public void setBrake() {
    //setting coast or brake mode, can also be done in Phoenix tuner
    falconFR.setNeutralMode(NeutralMode.Brake);
    falconFL.setNeutralMode(NeutralMode.Brake);
    falconBR.setNeutralMode(NeutralMode.Brake);
    falconBL.setNeutralMode(NeutralMode.Brake);
  }

  private double driveTrainP() {
    error = falconFL.getSelectedSensorPosition() - falconFR.getSelectedSensorPosition();
    //integral += error*.02;
    return DriveConstants.DRIVE_P*error;
  }

  public void driveStraight(double xSpeed) {
    drive.arcadeDrive(xSpeed, -driveTrainP());
  }

  /**
   * sets the speed of the drive train with arcade controls
   * @param xSpeed
   * @param zRotation
   */
  public void setDriveSpeed_Arcade(double xSpeed, double zRotation) {
    if (zRotation == 0 )
      driveStraight(xSpeed);
    drive.arcadeDrive(xSpeed, zRotation);
  }

  /**
   * set the speed of the drive train with tank controls (WIP)
   * @param lSpeed
   * @param rSpeed
   */
  public void setDriveSpeed_Tank(double lSpeed, double rSpeed) {
    drive.tankDrive(lSpeed, rSpeed);
  }

  /**
   * stops the drive train
   */
  public void stopRobot() {
    falconFR.set(ControlMode.PercentOutput, 0);
    falconFL.set(ControlMode.PercentOutput, 0);
  }

  /**
   * Returns the heading of the robot
   * @return The Heading of the robot in degrees, from -180 to 180
   */

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }


  /**
   * Returns the turn rate of the robot
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return -gyro.getRate();
  }

  /**
   * Gets the average distance of the two encoders
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (falconFR.getSelectedSensorPosition() + falconFL.getSelectedSensorPosition()) / 2.0;
  }

  /**
   * Zeroes the heading of the robot
   */
  public void zeroHeading() {
    gyro.reset();
  }

  public void resetEncoders() {
    falconFR.setSelectedSensorPosition(0);
    falconFL.setSelectedSensorPosition(0);
    falconBR.setSelectedSensorPosition(0);
    falconBL.setSelectedSensorPosition(0);  
  }


  /**
   * prints encoder values to the smart dashboard
   */
  public void printEncoderValues() {
    SmartDashboard.putNumber("FR pos", falconFR.getSelectedSensorPosition());
    //SmartDashboard.putNumber("BR pos", falconBR.getSelectedSensorPosition());
    SmartDashboard.putNumber("FL pos", falconFL.getSelectedSensorPosition());
    //SmartDashboard.putNumber("BL pos", falconBL.getSelectedSensorPosition());
    SmartDashboard.putNumber("FL vol", falconFL.getSelectedSensorVelocity());
    SmartDashboard.putNumber("FR Vol", falconFR.getSelectedSensorVelocity());
    SmartDashboard.putNumber("Angle", gyro.getAngle());
    SmartDashboard.putNumber("Pose X", odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("Pose Y", odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("Pose Rotation", odometry.getPoseMeters().getRotation().getDegrees());
  }

  public double getAvgPosition() {
    return (falconFR.getSelectedSensorPosition() + falconBR.getSelectedSensorPosition()) / 2;
  }

  //Returns
  public SimpleMotorFeedforward getFeedForward() {
    return feedforward;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public PIDController getLeftPidController() {
    return leftPidController;
  }

  public PIDController getRightPidController() {
    return rightPidController;
  }

  public Pose2d getPose() {
    return pose;
  }

  /**
   * Controls the left and right sides of the drive directly with voltages
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void setOutputVolts(double leftVolts, double rightVolts) {
    falconFR.setVoltage(leftVolts / 12);
    falconFL.setVoltage(leftVolts / 12);
    //drive.feed();
  } 

  /**
   * Resets the odometry to the specified pose.
   * 
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, getHeading());
  }

  @Override
  public void periodic() {

    pose = odometry.update(
      getHeading(), 
      falconFL.getSelectedSensorPosition() / 10.71 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60, 
      falconFR.getSelectedSensorPosition() / 10.71 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60); //TODO FIX ME
    printEncoderValues();
  }

}