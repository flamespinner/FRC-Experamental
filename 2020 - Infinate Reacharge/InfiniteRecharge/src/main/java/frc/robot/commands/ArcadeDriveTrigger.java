/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ArcadeDriveTrigger extends CommandBase {

  private DriveSubsystem m_drive;
  private DoubleSupplier m_xSpeed;
  private DoubleSupplier m_xSpeed2;
  private DoubleSupplier m_zRotation;
  /**
   * Creates a new WaccArcadeDrive.
   */
  public ArcadeDriveTrigger(DriveSubsystem drive, DoubleSupplier xSpeed, DoubleSupplier xSpeed2, 
                                            DoubleSupplier zRotation) {
    m_drive = drive;
    //right trigger drives fowards, left trigger drives backward
    m_xSpeed = xSpeed;
    m_xSpeed2 = xSpeed2;
    //needed to be inverted
    m_zRotation = zRotation;
 
    //declaring subsystem dependencies
    addRequirements(m_drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_drive.setDriveSpeed_Arcade(m_xSpeed2.getAsDouble() - m_xSpeed.getAsDouble(), 
    m_zRotation.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
