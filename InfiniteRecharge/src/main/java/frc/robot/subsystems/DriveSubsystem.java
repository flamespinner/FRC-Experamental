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
import frc.robot.Constants.DriveConstants;
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
   
   encoderReset();
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
   * prints encoder values to the smart dashboard
   */
  public void printEncoderValues() {
    SmartDashboard.putNumber("FR pos", falconFR.getSelectedSensorPosition());
    SmartDashboard.putNumber("BR pos", falconBR.getSelectedSensorPosition());
    SmartDashboard.putNumber("FL pos", falconFL.getSelectedSensorPosition());
    SmartDashboard.putNumber("BL pos", falconBL.getSelectedSensorPosition());
  }

  public double getAvgPosition() {
    return (falconFR.getSelectedSensorPosition() + falconBR.getSelectedSensorPosition()) / 2;
  }

  public void encoderReset() {
    falconFR.setSelectedSensorPosition(0);
    falconFL.setSelectedSensorPosition(0);
    falconBR.setSelectedSensorPosition(0);
    falconBL.setSelectedSensorPosition(0);
  }
}