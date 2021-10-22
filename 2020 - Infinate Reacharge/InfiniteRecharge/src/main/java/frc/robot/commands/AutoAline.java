/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.utils.Limelight;

public class AutoAline extends CommandBase {

//  private Limelight limelight;
  private DriveSubsystem m_driveSub;

  private double heading_error;
  private double distance_error;
  private double x;
  private double y; 
  private double steering_adjust = 0.0f;

  private double KpAim = -0.1f;
  private double KpDistance = -0.1f;
  private double min_aim_command = 0.05f;
  private double left_command = 0.0f;
  private double right_command = 0.0f;
  /**
   * Creates a new AutoAline.
   */
  public AutoAline(DriveSubsystem driveSub) {
    m_driveSub = driveSub;
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //x = limelight.getTx();
    //y = limelight.getTx();
    x = Limelight.getTx();
    y = Limelight.getTy();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //heading_error = -limelight.getTx();
    heading_error = -Limelight.getTx();
    distance_error = -Limelight.getTy();
    //distance_error = -limelight.getTx();

    if (x > 1.0) {
      steering_adjust = KpAim*heading_error - min_aim_command;
    } else if (x < 1.0) {
      steering_adjust = KpAim*heading_error + min_aim_command;
    }

    double distance_adjust = KpDistance * distance_error;

    left_command += steering_adjust + distance_adjust;
    right_command -= steering_adjust + distance_adjust;
    m_driveSub.setDriveSpeed_Tank(left_command, right_command);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driveSub.stopRobot();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
