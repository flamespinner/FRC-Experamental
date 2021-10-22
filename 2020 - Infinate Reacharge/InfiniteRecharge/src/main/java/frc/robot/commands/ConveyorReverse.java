/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.QueueConstants;
import frc.robot.subsystems.QueueSubsystem;

public class ConveyorReverse extends CommandBase {
  QueueSubsystem queueSub;

  /**
   * Creates a new ConveyorRun.
   */
  public ConveyorReverse(QueueSubsystem queueSub) {
    this.queueSub = queueSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(queueSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    queueSub.spinConveyor(QueueConstants.CONVEYOR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    queueSub.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
