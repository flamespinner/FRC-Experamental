// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class AutoCross extends CommandBase {
  /** Creates a new AutoCross. */
  private DriveSubsystem m_driveSub;
  public AutoCross() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
   
      m_driveSub.setDriveSpeed_Arcade(-.5, 0);
    

     

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_driveSub.setDriveSpeed_Arcade(-.5, 0);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSub.setDriveSpeed_Arcade(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(RobotContainer.driveSubsystem.getEncoder() < 5000){return false;}
    else {return true;}
  }
}
