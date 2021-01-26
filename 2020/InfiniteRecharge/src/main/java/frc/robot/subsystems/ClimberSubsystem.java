/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

/**
 * this subsystem sets up and directly manipulates the climber
 */
public class ClimberSubsystem extends SubsystemBase {

  //delcares and intializes climb motor
  private CANSparkMax winch = new CANSparkMax(ClimberConstants.WINCH_NEO_ID, MotorType.kBrushless);
  private WPI_TalonSRX climb = new WPI_TalonSRX(ClimberConstants.CLIMB_RED_ID);
  
  /**
   * Creates a new ClimberSubsystem.
   */
  public ClimberSubsystem() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * sets the speed of the climber to a constant speed
   */
  public void raiseClimber() {
    climb.set(ClimberConstants.CLIMB_SPEED);
  }

  /**
    * stops the climb motor
    */
  public void stopClimber() {
    climb.set(0);
  }

  public void lowerClimber() {
    climb.set(-ClimberConstants.CLIMB_SPEED);
  }

  public void startWinch() {
    winch.set(ClimberConstants.WINCH_SPEED);
  }

  public void stopWinch() {
    winch.set(0);
  }
}
