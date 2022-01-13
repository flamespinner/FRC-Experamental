/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Joystick m_stick;
  private static final int deviceID = 53; //setting CAN ID here
  private CANSparkMax m_motor;
  private CANEncoder m_encoder;
  private final WPI_TalonSRX RL1 = new WPI_TalonSRX(5);
  private final WPI_TalonSRX RL2 = new WPI_TalonSRX(9);
  private ShuffleboardTab tab = Shuffleboard.getTab("SmartDashboard");
  private NetworkTableEntry maxSpeed = 
    tab.add("Max Speed", 1)
      .withWidget(BuiltInWidgets.kNumberSlider)
      .withProperties(Map.of("min", 0, "max", 1))
      .getEntry();


  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {

    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless); //setting weather its a brushed or non brushed motor
    RL2.follow(RL1);
    RL1.setInverted(false);
    RL2.setInverted(InvertType.FollowMaster);
    m_encoder = m_motor.getEncoder();
    m_stick = new Joystick(1);
    }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }
  

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {

    m_motor.set(m_stick.getY());

    double max = maxSpeed.getDouble(0);
    //robotDrive.tankDrive(left * max, right * max);
    /**
     * Encoder position is read from a CANEncoder object by calling the
     * GetPosition() method.
     * 
     * GetPosition() returns the position of the encoder in units of revolutions
     */
    SmartDashboard.putNumber("Neo Encoder Position", m_encoder.getPosition());

    /**
     * Encoder velocity is read from a CANEncoder object by calling the
     * GetVelocity() method.
     * 
     * GetVelocity() returns the velocity of the encoder in units of RPM
     */
    SmartDashboard.putNumber("Neo Encoder Velocity", m_encoder.getVelocity());

    SmartDashboard.putNumber("Neo Tempature", m_motor.getMotorTemperature());
    SmartDashboard.putNumber("RedLine output Speed", RL1.getSelectedSensorVelocity());
    SmartDashboard.putNumber("RedLine output Pos", RL1.getSelectedSensorPosition());
    SmartDashboard.putNumber("Redline1 out%", RL1.getMotorOutputPercent());
    SmartDashboard.putNumber("Redline2 out%", RL2.getMotorOutputPercent());
  }
  
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
