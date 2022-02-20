// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants

public class DriveSubsystem extends SubsystemBase {

  private final WPI_TalonFX falconBL = new WPI_TalonFX(DriveConstants.Falcon_BL_ID);
  private final WPI_TalonFX falconFL = new WPI_TalonFX(DriveConstants.Falcon_FL_ID);
  private final WPI_TalonFX falconBR = new WPI_TalonFX(DriveConstants.Falcon_BR_ID);
  private final WPI_TalonFX falconFR = new WPI_TalonFX(DriveConstants.Falcon_FR_ID);

  private final TalonFXConfiguration fxconfig = new TalonFXConfiguration();

  private final MotorControllerGroup SCG_R = new MotorControllerGroup(falconFR, falconBR);
  private final MotorControllerGroup SCG_L = new MotorControllerGroup(falconFL, falconBL);

  private final DifferentialDrive drive = new DifferentialDrive(SCG_L, SCG_R);

  private double error;
  private double integral;


  public DriveSubsystem() {
    setBrake();
  
    falconFR.configOpenloopRamp(1.0);
    falconFR.configClosedLoopRamp(0);

    falconFL.configOpenloopRamp(1.0);
    falconFL.configClosedLoopRamp(0);

    falconBR.configOpenloopRamp(1.0);
    falconBR.configClosedLoopRamp(0);

    falconBL.configOpenloopRamp(1.0);
    falconBL.configClosedLoopRamp(0);

    falconBR.follow(falconFR);
    falconBL.follow(falconFL);

    falconBR.setInverted(InvertType.FollowMaster);
    falconBL.setInverted(InvertType.FollowMater);

    fxConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;

    encoderReset();
  }

}
