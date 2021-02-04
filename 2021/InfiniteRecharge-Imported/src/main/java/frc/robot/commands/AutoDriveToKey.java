/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDriveToKey extends CommandBase {

  private DriveSubsystem drive;
  private int inversion; 
  /**
   * Creates a new AutoCrossLine.
   */
  public AutoDriveToKey(DriveSubsystem driveSub, int inversion) {
    drive = driveSub; 
    this.inversion = inversion;
    drive.encoderReset();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setDriveSpeed_Arcade(inversion*.5, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.setDriveSpeed_Arcade(inversion*.5, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.setDriveSpeed_Arcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //SmartDashboard.putNumber("auto position", drive.getAvgPosition());
    //SmartDashboard.putBoolean("auto pos boolean", (drive.getAvgPosition() <= -9));
    return drive.getAvgPosition() <= (-14 / DriveConstants.TIC_FT);
  }
}
