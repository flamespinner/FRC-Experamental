// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/**Here we are creating our DriveTrain Subsystem. Here motor definitions (if not in a seperate class such as OI), drive train sensors etc should live */

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  //defining encoder counts per wheel revolution
  //1 rotation = 1440.0
  private static final double kCountsPerRevolution = 1440.0; 
  //setting the Diameter of the wheel in INCHES
  //NOTE: please make sure to label variables if its a distance or messurement. Labeled either Shorthand (mm, in, cm etc) or long hand (inch, etc)
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  // Here we are defining left and right motors. Assigning them to Sparks on PWM Ports 0 and 1.
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  // Here we are defining left and Right build in Encoders. ...new Encoder(DIO_CHANNEL_A, DIO_CHANNEL_B)
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  // Here we are creating your DifferentialDrive. Basically this is your full Drive definition. It needs your left motor and right motor variables.
  // on a robot with multi drive motors, you would define motor groups here instead of individual motors
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // Use inches as unit for encoder distances
    // These are defined above in your Drivebase Class
    // Doing math for Encoder Distance calculations
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
    resetEncoders();
  }

  //creating arcadeDrive
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  //call resetEnoders() to reset both encoders to 0
  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  //call getLeftEncoderCount() to get left encoder value
  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  //call getRightEncoderCount() to get right encoder value
  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  //call to get left encoder distance in inches
  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  //call to get right ecnoder distance in inches
  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  //call to get the average of both encoders
  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
