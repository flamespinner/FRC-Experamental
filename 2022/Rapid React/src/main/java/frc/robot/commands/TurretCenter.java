// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.TurretSubsystem;

public class TurretCenter extends CommandBase {
  /** Creates a new TurretCenter. */
  public TurretCenter() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (TurretSubsystem.m_encoder.getPosition() > 5)
    {
      RobotContainer.turretSubsystem.turnTurret(-ShooterConstants.centering_speed);
    
    }

    else if (TurretSubsystem.m_encoder.getPosition() < -5) {
      RobotContainer.turretSubsystem.turnTurret(ShooterConstants.centering_speed);
    }

    else {new TurretNeutral(new TurretSubsystem());}



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new TurretNeutral(new TurretSubsystem());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
