/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSub extends SubsystemBase {

  private final SwerveModule FR = new SwerveModule(
                              DriveConstants.DRIVE_FR, DriveConstants.TURNING_FR, 
                              DriveConstants.DRIVE_FR_ENC_INVERTED, DriveConstants.TURNING_FR_ENC_INVERTED);
  private final SwerveModule BR = new SwerveModule(
                              DriveConstants.DRIVE_BR, DriveConstants.TURNING_BR, 
                              DriveConstants.DRIVE_BR_ENC_INVERTED, DriveConstants.TURNING_BR_ENC_INVERTED);
  private final SwerveModule FL = new SwerveModule(
                              DriveConstants.DRIVE_FL, DriveConstants.TURNING_FL, 
                              DriveConstants.DRIVE_FL_ENC_INVERTED, DriveConstants.TURNING_FL_ENC_INVERTED);
  private final SwerveModule BL = new SwerveModule(
                              DriveConstants.DRIVE_BL, DriveConstants.TURNING_BL, 
                              DriveConstants.DRIVE_BL_ENC_INVERTED, DriveConstants.TURNING_BL_ENC_INVERTED);
                              
  private final Gyro m_gyro = new ADXRS450_Gyro();

  // Locations for the swerve drive modules relative to the robot center.
Translation2d m_frontLeftLocation = new Translation2d(DriveConstants.FL_MODULE_POS_X, DriveConstants.FL_MODULE_POS_Y);
Translation2d m_frontRightLocation = new Translation2d(DriveConstants.FR_MODULE_POS_X, DriveConstants.FR_MODULE_POS_Y);
Translation2d m_backLeftLocation = new Translation2d(DriveConstants.BL_MODULE_POS_X, DriveConstants.BL_MODULE_POS_Y);
Translation2d m_backRightLocation = new Translation2d(DriveConstants.BR_MODULE_POS_X, DriveConstants.BR_MODULE_POS_Y);

// Creating my kinematics object using the module locations
SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
  m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
);

  /**
   * Creates a new ExampleSubsystem.
   */
  public DriveSub() {

  }
  /**
   * 
   * @param xSpeed (m/sec)
   * @param ySpeed (m/sec)
   * @param rotSpeed (rad/sec)
   */
  public void sDrive(double xSpeed, double ySpeed, double rotSpeed) {

    ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);

    // Convert to module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

    // Front left module state
    SwerveModuleState frontLeft = moduleStates[0];

    // Front right module state
    SwerveModuleState frontRight = moduleStates[1];

    // Back left module state
    SwerveModuleState backLeft = moduleStates[2];

    // Back right module state
    SwerveModuleState backRight = moduleStates[3];

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
