// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Array;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;


public class ShooterSubsystem extends SubsystemBase {
  
// Creates a BangBangController
BangBangController BangBang = new BangBangController(); //instantiartes bangbang motor controller
private final WPI_TalonFX falconShooter = new WPI_TalonFX(ShooterConstants.FALCON_shooter_ID); 
SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(ShooterConstants.kS, ShooterConstants.kV, ShooterConstants.kA);
public double setpoint = 0.0; 
public double angle;
public double velocity;
static WPI_TalonSRX hoodMotor;
public double ShooterSpeed = falconShooter.getSelectedSensorVelocity();


  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    setCoast();
    hoodMotor = new WPI_TalonSRX(ShooterConstants.hood_motor_ID);
  }
  
  public void ShootBangBang() {
    //setpoint = getSetpoint();
    setpoint = 1200;
    falconShooter.set(BangBang.calculate(falconShooter.getSelectedSensorPosition(), (setpoint) + 0.9 * feedforward.calculate(setpoint)));
   //bang bang based on size of d from the hub

  }

  public void ShooterStop() {
    falconShooter.set(0);
  }

  public double getSetpoint () {
    double speedToGet = getV();
    setpoint = speedToGet; //needs math here
    return setpoint;
  }

  public double getAngle(){
    double d = TurretSubsystem.getDistance();
    double v = getV(); //m/s
    double beta = Math.atan(Math.abs((ShooterConstants.hub_height-ShooterConstants.limelight_height)/(d)));
    double dist = Math.sqrt(Math.pow(ShooterConstants.hub_height-ShooterConstants.limelight_height, 2) + Math.pow(d, 2));
    double angleTopPart = dist * 9.81 * Math.pow(Math.cos(beta), 2);
    double angleBottomPart = Math.pow(v, 2) + Math.sin(beta) + (beta / 2);
    double angle = .5 * Math.asin(angleTopPart / angleBottomPart);

    
    //math
    SmartDashboard.putNumber("angle of hood", angle);
    return angle;

  }

  public double getV(){
    double d = TurretSubsystem.getDistance();
    double speedToGet;
    if (d < .5) {speedToGet = 7.101;}
    else if (d < 1) {speedToGet = 6.614;}
    else if (d < 1.5) {speedToGet = 6.797;}
    else if (d < 2) {speedToGet = 6.797;}
    else if (d < 2.5) {speedToGet = 7.132;}
    else if (d < 3) {speedToGet = 7.407;}
    else if (d < 3.5) {speedToGet = 7.711;}
    else if (d < 4) {speedToGet = 8.138;}
    else if (d < 4.5) {speedToGet = 8.443;}
    else if (d < 5) {speedToGet = 8.63;}
    else if (d < 5.5) {speedToGet = 8.9;}
    else if (d < 6) {speedToGet = 9.14;}
    else if (d < 6.5) {speedToGet = 9.33;}
    else if (d < 7) {speedToGet = 9.45;}
    else if (d < 7.5) {speedToGet = 9.81;}
    else if (d < 8) {speedToGet = 10.06;}
    else if (d < 8.5) {speedToGet = 10.24;}
    else if (d < 9) {speedToGet = 10.49;}
    else {speedToGet = 8.5;}
    ///math
    SmartDashboard.putNumber("Flywheel Speed Needed", setpoint);
    return speedToGet;
  }

  public void alignHood() {
    double angle = getAngle();
    double currentEncoder = hoodMotor.getSelectedSensorPosition();
    double encoderCountRequired = angle * ShooterConstants.hood_encoder_ratio;
    SmartDashboard.putNumber("Hood Encoder Needed", encoderCountRequired);
    if (currentEncoder < (encoderCountRequired - ShooterConstants.min_command)) {
      adjustHood(.5); //to test speed and inversion
    }

    else if (currentEncoder > (encoderCountRequired + ShooterConstants.min_command)) {
      adjustHood(-.5); //to test speed and inversion
    }

    else {
      adjustHood(0);
    }

  }

  public void adjustHood(double speed) {
    hoodMotor.set(speed);
  }


  public void setCoast() {
    //setting coast or brake mode, can also be done in Phoenix tuner
    falconShooter.setNeutralMode(NeutralMode.Coast);
  }

  public void setBrake() {
    //setting coast or brake mode, can also be done in Phoenix tuner
    falconShooter.setNeutralMode(NeutralMode.Brake);
  }




  @Override
  public void periodic() {
    SmartDashboard.putNumber("Falcon Speed", ShooterSpeed);
    // This method will be called once per scheduler run
  }
}
