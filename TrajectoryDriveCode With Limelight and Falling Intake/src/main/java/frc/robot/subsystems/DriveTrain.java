// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveTrain extends SubsystemBase {
  /** Creates a new DriveTrain. */
WPI_TalonFX leftFront = new WPI_TalonFX(DriveConstants.LeftFront);
WPI_TalonFX leftRear = new WPI_TalonFX(DriveConstants.LeftRear);
WPI_TalonFX rightFront = new WPI_TalonFX(DriveConstants.RightFront);
WPI_TalonFX rightRear = new WPI_TalonFX(DriveConstants.RightRear);
SpeedControllerGroup leftSide = new SpeedControllerGroup(leftFront, leftRear);
SpeedControllerGroup rightSide = new SpeedControllerGroup(rightFront, rightRear);
DifferentialDrive drive = new DifferentialDrive(leftSide, rightSide);
static final byte navx_rate = 127;
AHRS navx = new AHRS(SPI.Port.kMXP, navx_rate);
DifferentialDriveOdometry odometry;




 public DriveTrain() {
  leftFront.setInverted(false);
  leftRear.setInverted(false);
  rightFront.setInverted(true);
  rightRear.setInverted(true);    
  leftFront.setNeutralMode(NeutralMode.Brake);
  leftRear.setNeutralMode(NeutralMode.Brake);
  rightFront.setNeutralMode(NeutralMode.Brake);
  rightRear.setNeutralMode(NeutralMode.Brake);
  odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  resetOdometry(new Pose2d());
  }

  @Override
  public void periodic() {
    odometry.update(myRotation(), leftFront.getSelectedSensorPosition()*DriveConstants.Conversion,
    rightFront.getSelectedSensorPosition()*DriveConstants.Conversion);
        // This method will be called once per scheduler run
    SmartDashboard.putNumber("my X Pose:",odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("my Y Pose:",odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("my Heading:",getHeading());
    SmartDashboard.putNumber("my leftEnc:",leftFront.getSelectedSensorPosition());
    SmartDashboard.putNumber("my rightEnc:",rightFront.getSelectedSensorPosition());
  }

  public Pose2d getPose(){
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds(){
    return new DifferentialDriveWheelSpeeds(leftFront.getSelectedSensorVelocity()*DriveConstants.Conversion,
    rightFront.getSelectedSensorVelocity()*DriveConstants.Conversion);
  }

  public void resetOdometry(Pose2d pose){
    resetEncoders();
    zeroHeading();
    odometry.resetPosition(pose, navx.getRotation2d());
  }

  public void arcadeDrive (double fwd, double rot){
    drive.arcadeDrive(fwd, rot);
  }

  public void tankDriveVolts(double leftVolts, double rightVolts){
    leftSide.setVoltage(leftVolts);
    rightSide.setVoltage(rightVolts);
    drive.feed();
  }

  public void resetEncoders(){
    leftFront.setSelectedSensorPosition(0);
    rightFront.setSelectedSensorPosition(0);
  }

  public double getAverageEncoderDistance(){
    return ((leftFront.getSelectedSensorPosition()+rightFront.getSelectedSensorPosition())/2.0)
    *DriveConstants.Conversion;
  }

  /**
   * Set the max output of the drive. 
   * @param setMaxOutput
   */
  public void setMaxOutput(double maxOutput){
    drive.setMaxOutput(maxOutput);
      }

  public void zeroHeading(){
    navx.zeroYaw();
  }

  public double getHeading(){
    return navx.getRotation2d().getDegrees();
  }

  public Rotation2d myRotation(){
    return Rotation2d.fromDegrees(getHeading());
  }

  public double getTurnRate(){
    return navx.getRate();
  }

public static boolean WPI_TalonFX(int leftfront2) {
	return false;
}
}
