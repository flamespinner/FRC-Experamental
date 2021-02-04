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

public class ReverseQueue extends CommandBase {

  private QueueSubsystem m_queueSub;
  /**
   * Creates a new ReverseQueue.
   */
  public ReverseQueue(QueueSubsystem queueSub) {
    m_queueSub = queueSub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_queueSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_queueSub.spinConveyor(QueueConstants.CONVEYOR_SPEED);
    m_queueSub.spinIndexer(QueueConstants.INDEXER_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_queueSub.stopConveyor();
    m_queueSub.stopIndexer();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
