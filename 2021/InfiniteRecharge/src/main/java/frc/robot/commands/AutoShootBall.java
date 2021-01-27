/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoShootBall extends CommandBase {

  private ShooterSubsystem m_shootSub;
  private Timer time = new Timer();
  private double startTime;
  /**
   * Creates a new ShootBall.
   */
  public AutoShootBall(ShooterSubsystem shootSub) {
    m_shootSub = shootSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_shootSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = time.getFPGATimestamp();
    m_shootSub.setSetpoint(ShooterConstants.SHOOTER_SETPOINT);
    RobotContainer.shooting = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shootSub.shootPIControl();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shootSub.stopShooter();
    RobotContainer.shooting = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (time.getFPGATimestamp() - startTime) >= ShooterConstants.AUTO_SHOOT_TIME;
  }
}
