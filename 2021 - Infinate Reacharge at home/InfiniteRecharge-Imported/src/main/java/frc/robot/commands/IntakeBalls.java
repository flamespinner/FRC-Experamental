/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.QueueConstants;
import frc.robot.subsystems.QueueSubsystem;

/**
 * runs the code that controls the intake
 */
public class IntakeBalls extends CommandBase {

  private QueueSubsystem m_queue;

  /**
   * Creates a new IntakeBalls.
   */
  public IntakeBalls(QueueSubsystem queue) {
    m_queue = queue; 
    // declaring subsystem dependencies
    addRequirements(m_queue);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //turns on the queue
    m_queue.spinIntake(-QueueConstants.INTAKE_SPEED); //INVERTED CORRECT
    m_queue.spinIndexer(-QueueConstants.INDEXER_SPEED);
    m_queue.spinConveyor(QueueConstants.CONVEYOR_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.IntakeBalls
  @Override
  public void execute() {
    //contiues to take user input
    if(!m_queue.lineBreakBroken() && !RobotContainer.shooting) {
      m_queue.stopConveyor();
    }
    else if(!m_queue.lineBreakBroken() && RobotContainer.shooting) { 
      m_queue.spinConveyor(-QueueConstants.CONVEYOR_SPEED);
    }
    else {
      m_queue.spinConveyor(-QueueConstants.CONVEYOR_SPEED);
    }
    
    SmartDashboard.putBoolean("Linebreak Broken", m_queue.lineBreakBroken());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //turns off queue
    m_queue.stopIntake();
    m_queue.stopIndexer();
    m_queue.stopConveyor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
