/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final int deviceID = 51; //setting CAN ID here
  private CANSparkMax m_motor;
  private CANEncoder m_encoder;

  private TalonFX talonFX = new TalonFX(40);
  
  private TalonFXConfiguration fxConfig = new TalonFXConfiguration();

  private Joystick _Joystick = new Joystick(0);

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    fxConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; //Selecting Feedback Sensor

    m_motor = new CANSparkMax(deviceID, MotorType.kBrushless); //setting weather its a brushed or non brushed motor

    m_encoder = m_motor.getEncoder();

    talonFX.configAllSettings(fxConfig); 
    talonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 50);

    talonFX.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    talonFX.configOpenloopRamp(0.5); // 0.5 seconds from neutral to full output (during open-loop control)
    talonFX.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)
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

    //outputs things to SmartDashboard/Shuffleboard
    SmartDashboard.putNumber("talon ID 40 pos", talonFX.getSelectedSensorPosition());
    SmartDashboard.putNumber("talon ID 40 velocity", talonFX.getSelectedSensorVelocity());

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
    double stick = _Joystick.getRawAxis(1) * -1; //makes forward positive
    talonFX.set(ControlMode.PercentOutput, stick);

    m_motor.set(m_stick.getX());
    SmartDashboard.putNumber("Neo Encoder Position", m_encoder.getPosition());
    SmartDashboard.putNumber("Neo Encoder Velocity", m_encoder.getVelocity());

    SmartDashboard.putNumber("Neo Tempature", m_motor.getMotorTemperature());

  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}