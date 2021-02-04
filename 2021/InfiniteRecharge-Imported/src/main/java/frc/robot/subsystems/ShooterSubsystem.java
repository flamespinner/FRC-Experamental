/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
/**
 * this subsystem sets up and directly manipulates the high goal shooter
 */
public class ShooterSubsystem extends SubsystemBase {

  //declaring and intializing shooter motor
  private CANSparkMax shooterMotor = new CANSparkMax(ShooterConstants.SHOOTER_SPARK_ID, MotorType.kBrushless);
  private CANEncoder enc = shooterMotor.getEncoder();

  public Servo servo;
  private double integral, setpoint = 0;
  private double error;
  /**
   * Creates a new ShooterSubsystem.
   */
  public ShooterSubsystem() {
    servo = new Servo(ShooterConstants.LIMELIGHT_SERVO_ID);
    setServo(ShooterConstants.LIMELIGHT_ANGLE_SETPOINT);

    shooterMotor.setInverted(false); //invert motor
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public void setServo(double degrees) {
    servo.setAngle(degrees);
  }

  public void setSetpoint(double set) {
    setpoint = set;
  }

  /**
   * sets the speed of the shooter to a constant speed
   */
  public void shootBall() {
    shooterMotor.set(ShooterConstants.HIGH_GOAL_SPEED);
   }

   public void reverseShooter() {
     shooterMotor.set(-ShooterConstants.HIGH_GOAL_SPEED);
   }

   /**
    * stops the shooter motor
    */
   public void stopShooter() {
     shooterMotor.set(0);
   }

   private double shooterPI() {
     error = setpoint - enc.getVelocity();
     //integral += error*.02;
     return ShooterConstants.SHOOTER_P*error;
   }

   public void shootPIControl() {
     shooterMotor.set((shooterPI() + setpoint) / ShooterConstants.SHOOTER_MAX_VELOCITY);
     SmartDashboard.putNumber("Shooter PI", shooterPI()+ setpoint);
     SmartDashboard.putNumber("Shooter Power", (shooterPI() + setpoint) / ShooterConstants.SHOOTER_MAX_VELOCITY);
   }

   public double getMotorTemp() {
     return shooterMotor.getMotorTemperature();
   }
}
