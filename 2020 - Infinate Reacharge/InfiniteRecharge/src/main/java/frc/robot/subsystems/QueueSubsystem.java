/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.QueueConstants;

/**
 * this subsystem sets up and directly manipulates the mechanisms to queue balls into the shooter
 * - intake, indexer, conveyor -
 */
public class QueueSubsystem extends SubsystemBase {

  //delcaring and intializing intake motor
  private WPI_TalonSRX intake = new WPI_TalonSRX(QueueConstants.INTAKE_RED_ID);
  //delcaring and intializing indexer motors
  private WPI_TalonSRX indexer1 = new WPI_TalonSRX(QueueConstants.INDEXER_RED_ID_1);
  private WPI_TalonSRX indexer2 = new WPI_TalonSRX(QueueConstants.INDEXER_RED_ID_2);
  private WPI_TalonSRX indexerBelt = new WPI_TalonSRX(QueueConstants.INDEXER_BOTTOM_RED_ID);
  //delcaring and intializing conveyor motor
  private WPI_TalonSRX conveyor = new WPI_TalonSRX(QueueConstants.CONVEYOR_RED_ID_1);

  private DigitalInput transmitter = new DigitalInput(QueueConstants.LINEBREAK_TRANSMITTER_ID);
  private DigitalInput reciver = new DigitalInput(QueueConstants.LINEBREAK_RECIVER_ID);

  /**
   * Creates a new QueueSubsystem.
   */
  public QueueSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean lineBreakBroken() {
    return reciver.get(); 
  } 

  /**
   * sets the speed of the intake to a constant speed
   */
  public void spinIntake() {
    intake.set(QueueConstants.INTAKE_SPEED);
  }

  /**
   * sets the speed of the intake to inputted speed
   */
  public void spinIntake(double speed) {
    intake.set(speed);
  }

  /**
    * stops the intake motor
    */
  public void stopIntake() {
    intake.set(0);
  }
  
  /**
   * sets the speed of the indexer to a constant speed
   */
  public void spinIndexer() {
      indexer1.set(-QueueConstants.INDEXER_SPEED);
      indexer2.set(-QueueConstants.INDEXER_SPEED*.75);
      indexerBelt.set(-QueueConstants.INDEXER_BOTTOM_SPEED);
  }

  public void spinIndexerBelt() {
    indexerBelt.set(QueueConstants.INDEXER_BOTTOM_SPEED);
  }

  public void stopIndexerBelt() {
    indexerBelt.set(0);
  }
  /**
   * sets the speed of the indexer to inputted speed
   */
  public void spinIndexer(double speed) {
    indexer1.set(-speed);
    indexer2.set(-speed*.75);
    indexerBelt.set(-speed);
  }

  /**
    * stops the indexer motors
    */
  public void stopIndexer() {
      indexer1.set(0);
      indexer2.set(0);
      indexerBelt.set(0);
  }
  
  /**
   * sets the speed of the conveyor to a constant speed
   */
  public void spinConveyor() {
      conveyor.set(QueueConstants.CONVEYOR_SPEED);
  }

  /**
   * sets the speed of the conveyor to inputted speed
   */
  public void spinConveyor(double speed) {
    conveyor.set(speed);
  }
  /**
    * stops the conveyor motor
    */
  public void stopConveyor() {
      conveyor.set(0);
  }
}
