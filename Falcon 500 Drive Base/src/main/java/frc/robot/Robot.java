/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.StringJoiner;

//import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  /**Drivebase Code */
  private final TalonFX falconBL = new TalonFX(42); //Right
  private final TalonFX falconFL = new TalonFX(43); //Right
  private final TalonFX falconBR = new TalonFX(41); //Left
  private final TalonFX falconFR = new TalonFX(44); //Left

  private double i_falconBL;
  private double i_falconBR;
  private double i_falconFL;
  private double i_falconFR;

  /**End of Drivebase Code */
  
  private final TalonFXConfiguration fxConfig = new TalonFXConfiguration();
  private XboxController xbox = new XboxController(3); 

  private AnnyDDrive drive = new AnnyDDrive(falconFR, falconBR, falconFL, falconBL);
  private AnalogGyro gyro = new AnalogGyro(0);

  private static final double kAngleSetpoint = 0.0;
  private static final double kP = 0.005; // propotional turning constant

	// gyro calibration constant, may need to be adjusted;
	// gyro value of 360 is set to correspond to one full revolution
  private static final double kVoltsPerDegreePerSecond = 0.0128;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    gyro.setSensitivity(kVoltsPerDegreePerSecond);
    //gyro.calibrate();
    gyro.reset();
   //falconFR.configIntegratedSensorInitializationStrategy(fxConfig.initializationStrategy.BootToAbsolutePosition);
    //fxConfig.initializationStrategy.BootToZero

    /**setting coast or brake mode, can also be done in Phoenix tuner */
//    talonFX.setNeutralMode(NeutralMode.Brake);
    falconFR.setNeutralMode(NeutralMode.Brake);
    falconFL.setNeutralMode(NeutralMode.Brake);
    falconBR.setNeutralMode(NeutralMode.Brake);
    falconBL.setNeutralMode(NeutralMode.Brake);
    /**end of setting modes */

    /**set ramp */
    falconFR.configOpenloopRamp(0.5); // 0.5 seconds from neutral to full output (during open-loop control)
    falconFR.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)

    falconFL.configOpenloopRamp(0.5); // 0.5 seconds from neutral to full output (during open-loop control)
    falconFL.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)

    falconBL.configOpenloopRamp(0.5); // 0.5 seconds from neutral to full output (during open-loop control)
    falconBL.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)

    falconBR.configOpenloopRamp(0.5); // 0.5 seconds from neutral to full output (during open-loop control)
    falconBR.configClosedloopRamp(0); // 0 disables ramping (during closed-loop control)
    /**end set ramp */

    /**Drive Base Code Start */
    falconBR.follow(falconFR); //talonBR follows TalonFR
    falconBL.follow(falconFL); //talonBL follows TalonFR 

    falconFR.setInverted(true); //set to invert falconFR.. CW/CCW.. Green = forward (motor led)
    falconBR.setInverted(InvertType.FollowMaster); //matches whatever falconFR is

    falconFL.setInverted(true); //set to invert falconFL.. CW/CCW.. Green = foward (motor led)
    falconBL.setInverted(InvertType.FollowMaster); //matches whatever falcon FL is
    /**Drive Base Code End */

    /**Encoder Code Start */
    fxConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; //Selecting Feedback Sensor

//    talonFX.configAllSettings(fxConfig); 
//    talonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 50);

//    talonFX.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    /**Encoder Code End */

    /**other */
    //talonFX.configVoltageCompSaturation(11); //voltage comparison
    /**end other */
    i_falconBL = falconBL.getSelectedSensorPosition() * ((Math.PI)/2014)/10.75;
    i_falconBR = falconBR.getSelectedSensorPosition() * ((Math.PI)/2014)/10.75;
    i_falconFL = falconFL.getSelectedSensorPosition() * ((Math.PI)/2014)/10.75;
    i_falconFR = falconFR.getSelectedSensorPosition() * ((Math.PI)/2014)/10.75;
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
    double turningValue = (kAngleSetpoint - gyro.getAngle()) * kP;
    turningValue = Math.copySign(turningValue, xbox.getY(Hand.kLeft));
    drive.arcadeDrive(xbox.getY(Hand.kLeft), turningValue); 

    /*drive.arcadeDrive(Math.abs(xbox.getTriggerAxis(Hand.kLeft)) - Math.abs((xbox.getTriggerAxis(Hand.kRight))), 
                      -xbox.getX(Hand.kLeft));*/

    //outputs things to SmartDashboard/Shuffleboard
    //SmartDashboard.putNumber("Angle", imu.getGyroAngleX());
    SmartDashboard.putNumber("FR pos", getRealPosition_FT(falconFR));
    SmartDashboard.putNumber("BR pos", getRealPosition_FT(falconBR));
    SmartDashboard.putNumber("FL pos", getRealPosition_FT(falconFL));
    SmartDashboard.putNumber("BL pos", getRealPosition_FT(falconBL));
    SmartDashboard.putNumber("Angle", gyro.getAngle());
    SmartDashboard.putNumber("Rate", gyro.getRate());

/*    SmartDashboard.putNumber("imu angle", imu.getAngle()); 
    SmartDashboard.putNumber("imu angle X", imu.getGyroAngleX()); 
    SmartDashboard.putNumber("imu angle Y", imu.getGyroAngleY()); 
    SmartDashboard.putNumber("imu angle Z", imu.getGyroAngleZ()); */

  }

  private double getRealPosition_FT(TalonFX talon) throws IllegalArgumentException{
    if (talon == falconBL) 
        return Math.abs(i_falconBL - falconBL.getSelectedSensorPosition() * ((Math.PI)/2014)/10.75);
    else if (talon == falconBR) 
        return Math.abs(i_falconBR - falconBR.getSelectedSensorPosition() * ((Math.PI)/2014)/10.75);
    else if (talon == falconFL) 
        return Math.abs(i_falconFL - falconFL.getSelectedSensorPosition() * ((Math.PI)/2014)/10.75);
    else if (talon == falconFR) 
        return Math.abs(i_falconFR - falconFR.getSelectedSensorPosition() * ((Math.PI)/2014)/10.75);
    else 
        throw new IllegalArgumentException("cannot retrive encoder data from an invailid talon");
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

//    final double xstick = xbox.getRawAxis(1) * -1;
//    final double xstick1 = xbox.getRawAxis(2) * -1;
  //    talonFX.set(ControlMode.PercentOutput, stick);
  //falconFR.set(ControlMode.PercentOutput, RT);
  //falconBR.set(ControlMode.PercentOutput, RT);

  //falconFL.set(ControlMode.PercentOutput, LT);
  //falconBL.set(ControlMode.PercentOutput, LT);
  }
  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
/**
 * <p> Diifrential Drive with TalonFX controllers
 * BR and BL follow FR and FL
 * 
 * @param FR, BR, FL, BL
 */
class AnnyDDrive extends RobotDriveBase implements Sendable, AutoCloseable {
  public static final double kDefaultQuickStopThreshold = 0.2;
  public static final double kDefaultQuickStopAlpha = 0.1;

  private static int instances;

  private final TalonFX m_FR;
  private final TalonFX m_BR;
  private final TalonFX m_FL;
  private final TalonFX m_BL;

  private double m_quickStopThreshold = kDefaultQuickStopThreshold;
  private double m_quickStopAlpha = kDefaultQuickStopAlpha;
  private double m_quickStopAccumulator;
  private double m_rightSideInvertMultiplier = -1.0;
  private boolean m_reported;

  public AnnyDDrive(TalonFX FR, TalonFX BR, TalonFX FL, TalonFX BL) {
      verify(FR, BR, FL, BL);
      m_FR = FR;
      m_BR = BR;
      m_FL = FL;
      m_BL = BL;
      SendableRegistry.addChild(this, FR);
      SendableRegistry.addChild(this, FL);
      SendableRegistry.addChild(this, BR);
      SendableRegistry.addChild(this, BL);
      m_BR.follow(FR);
      m_BL.follow(FL);
      instances++;
      SendableRegistry.addLW(this, "AnnyDDrive", instances);
  }
  @SuppressWarnings("PMD.AvoidThrowingNullPointerException")
  private void verify(TalonFX FR, TalonFX BR, TalonFX FL, TalonFX BL) {
      if (FR != null && BR != null && FL != null && BL != null)
          return; 
      StringJoiner joiner = new StringJoiner(", ");
      if (FR == null) {
          joiner.add("FR");
      }
      if (BR == null) {
          joiner.add("BR");
      }
      if (FL == null) {
          joiner.add("FL");
      }
      if (BL == null) {
          joiner.add("BL");
      }
      throw new NullPointerException(joiner.toString());  
  }
  @Override
  public void close() throws Exception {
      SendableRegistry.remove(this);
  }
  @SuppressWarnings("ParameterName")
  public void arcadeDrive(double xSpeed, double zRotation) {
      arcadeDrive(xSpeed, zRotation, true);
  }
  @SuppressWarnings("ParameterName")
  public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
      if (!m_reported) {
      HAL.report(tResourceType.kResourceType_RobotDrive,
                  tInstances.kRobotDrive2_DifferentialArcade, 2);
      m_reported = true;
      }

      xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
      xSpeed = applyDeadband(xSpeed, m_deadband);

      zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
      zRotation = applyDeadband(zRotation, m_deadband);

      // Square the inputs (while preserving the sign) to increase fine control
      // while permitting full power.
      if (squareInputs) {
      xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
      zRotation = Math.copySign(zRotation * zRotation, zRotation);
      }

      double leftMotorOutput;
      double rightMotorOutput;

      double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

      if (xSpeed >= 0.0) {
      // First quadrant, else second quadrant
      if (zRotation >= 0.0) {
          leftMotorOutput = maxInput;
          rightMotorOutput = xSpeed - zRotation;
      } else {
          leftMotorOutput = xSpeed + zRotation;
          rightMotorOutput = maxInput;
      }
      } else {
      // Third quadrant, else fourth quadrant
      if (zRotation >= 0.0) {
          leftMotorOutput = xSpeed + zRotation;
          rightMotorOutput = maxInput;
      } else {
          leftMotorOutput = maxInput;
          rightMotorOutput = xSpeed - zRotation;
      }
      }

      m_FL.set(ControlMode.PercentOutput, MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * m_maxOutput);
      double maxOutput = m_maxOutput * m_rightSideInvertMultiplier;
      m_FR.set(ControlMode.PercentOutput, MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * maxOutput);

      feed();
  }
  @Override
  public void initSendable(SendableBuilder builder) {
      // TODO Auto-generated method stub

  }

  @Override
  public void stopMotor() {
      m_BL.set(ControlMode.PercentOutput, 0);
      m_BR.set(ControlMode.PercentOutput, 0);
      m_FL.set(ControlMode.PercentOutput, 0);
      m_FR.set(ControlMode.PercentOutput, 0);
      feed();
  }

  @Override
  public String getDescription() {
      return "Anny's DifferentialDrive to work with TalonFX";
  }
}
