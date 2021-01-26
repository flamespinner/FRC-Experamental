/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSub;

public class SwerveDriveCommand extends CommandBase {
  private DriveSub m_drive;
  private double m_xSpeed;
  private double m_ySpeed;
  private double m_rotSpeed;
  /**
   * Creates a new SwerveDriveCommand.
   */
  public SwerveDriveCommand(DriveSub drive, double xSpeed, double ySpeed, double rotSpeed) {
    m_drive = drive;
    m_xSpeed = xSpeed;
    m_ySpeed = ySpeed;
    m_rotSpeed = rotSpeed;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.sDrive(m_xSpeed/DriveConstants.MAX_SPEED_MperSEC, 
                   m_ySpeed/DriveConstants.MAX_SPEED_MperSEC, 
                   m_rotSpeed/DriveConstants.MAX_SPEED_RADperSEC);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.sDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
