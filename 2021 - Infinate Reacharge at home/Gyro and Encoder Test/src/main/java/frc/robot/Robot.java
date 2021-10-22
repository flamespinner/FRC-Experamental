// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This is a sample program to demonstrate how to use a gyro sensor to make a
 * robot drive straight. This program uses a joystick to drive forwards and
 * backwards while the gyro is used for direction keeping.
 */
public class Robot extends TimedRobot {
  private static final double kAngleSetpoint = 0.0;
  private static final double kP = 0.005; // propotional turning constant

  // gyro calibration constant, may need to be adjusted;
  // gyro value of 360 is set to correspond to one full revolution
  private static final double kVoltsPerDegreePerSecond = 0.0128;

  // Defining Drive Talons
  private final WPI_TalonSRX leftFront = new WPI_TalonSRX(1); // TODO ID
  private final WPI_TalonSRX leftBack = new WPI_TalonSRX(2); // TODO ID
  private final WPI_TalonSRX rightFront = new WPI_TalonSRX(3); // TODO ID
  private final WPI_TalonSRX rightBack = new WPI_TalonSRX(4); // TODO ID

  // define speed controller groups
  private SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftFront, leftBack);
  private SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightFront, rightBack);

  // define Differental Drive
  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

  // define Gyro Port
  private static final int kGyroPort = 0;

  // define Controller USB Port (in driver station)
  private final XboxController xbox = new XboxController(0);
  private final AnalogGyro m_gyro = new AnalogGyro(kGyroPort);

  @Override
  public void robotInit() {
    m_gyro.setSensitivity(kVoltsPerDegreePerSecond);
    Shuffleboard.getTab("Gyro Compass").add((Sendable) m_gyro);
  }

  /**
   * The motor speed is set from the joystick while the RobotDrive turning value is assigned from
   * the error between the setpoint and the gyro angle.
   */
  @Override
  public void teleopPeriodic() {
    double turningValue = (kAngleSetpoint - m_gyro.getAngle()) * kP;
    // Invert the direction of the turn if we are going backwards
    turningValue = Math.copySign(turningValue, xbox.getY());
    drive.arcadeDrive(xbox.getY(), turningValue);
    SmartDashboard.putNumber("Gyro Value", m_gyro.getAngle());
  }
}
