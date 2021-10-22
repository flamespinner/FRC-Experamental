/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {


  private WPI_TalonSRX FR = new WPI_TalonSRX(DriveConstants.FR_ID);
    private WPI_TalonSRX BR = new WPI_TalonSRX(DriveConstants.BR_ID);
    private WPI_TalonSRX FL = new WPI_TalonSRX(DriveConstants.FL_ID);
    private WPI_TalonSRX BL = new WPI_TalonSRX(DriveConstants.BL_ID);

    private SpeedControllerGroup SCGRight = new SpeedControllerGroup(FR, BR);
    private SpeedControllerGroup SCGLeft = new SpeedControllerGroup(FL, BL);
    
    private DifferentialDrive drive = new DifferentialDrive(SCGLeft, SCGRight);  


  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    
  }
  //method void 2 doubles
public void tankDrive(double leftDriveIntake, double rightDriveIntake){
  drive.tankDrive(leftDriveIntake, rightDriveIntake);

}
  @Override
  public void periodic() {
   
    
    // This method will be called once per scheduler run
  }
}
