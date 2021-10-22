/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class AutoCross extends CommandBase {
  private static DriveTrain m_driveSub;

  /**
   * Creates a new AutoDriveToWall.
   */
  public AutoCross(DriveTrain driveSub) {
    m_driveSub = driveSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    WPI_TalonFX leftFront = new WPI_TalonFX(DriveConstants.LeftFront);
    while (leftFront.getSelectedSensorPosition() < 2500) {
      m_driveSub.setDriveSpeed_Arcade(1, 0);
      }
    while (leftFront.getSelectedSensorPosition() > 0)
     {m_driveSub.setDriveSpeed_Arcade(-1, 0);}
   
     m_driveSub.setDriveSpeed_Arcade(0, 0);



    }

// Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSub.setDriveSpeed_Arcade(.5, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSub.setDriveSpeed_Arcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
