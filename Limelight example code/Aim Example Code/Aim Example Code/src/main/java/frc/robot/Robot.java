/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private final WPI_TalonSRX m_Left0 = new WPI_TalonSRX(5); // FL
  private final WPI_TalonSRX m_Left1 = new WPI_TalonSRX(3); // BL
  private final WPI_TalonSRX m_Right0 = new WPI_TalonSRX(5); // FR
  private final WPI_TalonSRX m_Right1 = new WPI_TalonSRX(8); // BR
  private SpeedControllerGroup m_LeftMotors = new SpeedControllerGroup(m_Left0, m_Left1);
  private SpeedControllerGroup m_RightMotors = new SpeedControllerGroup(m_Right0, m_Right1);
  private final DifferentialDrive m_Drive = new DifferentialDrive(m_LeftMotors, m_RightMotors);

  private final XboxController m_Controller = new XboxController(0);

  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
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
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry ts = table.getEntry("ts");
    NetworkTableEntry pipe = table.getEntry("getPipe");
    NetworkTableEntry tv = table.getEntry("tv");

    double x = tx.getDouble(0);
    double y = tx.getDouble(0);
    double area = ta.getDouble(0);
    double KpAim = -0.1f;
    double KpDistance = -0.1f;
    double min_aim_command = 0.05f;
    double left_command = 0.0f;
    double right_command = 0.0f;
    System.out.println("teleop");

    if (m_Controller.getAButton()) {
      System.out.println("Button A");
      double heading_error = -x;
      double distance_error = -y;
      double steering_adjust = 0.0f;
      
      if (x > 1.0) {
        steering_adjust = KpAim*heading_error - min_aim_command;
      } else if (x < 1.0) {
        steering_adjust = KpAim*heading_error + min_aim_command;
      }

      double distance_adjust = KpDistance * distance_error;

      left_command += steering_adjust + distance_adjust;
      right_command -= steering_adjust + distance_adjust;
      m_Drive.tankDrive(left_command, right_command);
    }



  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
