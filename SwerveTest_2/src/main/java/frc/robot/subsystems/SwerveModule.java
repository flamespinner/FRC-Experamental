/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Constants.ModuleConstants;

/**
 * Add your docs here.
 */
public class SwerveModule {
    private final TalonSRX driveMotor;
    private final TalonSRX turningMotor;

    private final PIDController drivePID = new PIDController(ModuleConstants.DRIVE_CONTROLLER_P, 0, 0); 
    private final ProfiledPIDController turningPID = 
                        new ProfiledPIDController(ModuleConstants.TURNING_CONTROLLER_P, 0, 0, 
                        new TrapezoidProfile.Constraints(ModuleConstants.MAX_ANGULAR_SPEED_RADperSEC, 
                                                        ModuleConstants.MAX_ANGULAR_ACCLERATION_RADperSECsquared));


    public SwerveModule(int driveMotorID, int turningMotorID, boolean driveEncoderReversed, boolean turningEncoderReversed) {
        driveMotor = new TalonSRX(driveMotorID);
        turningMotor = new TalonSRX(turningMotorID);
    
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
        turningMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

        driveMotor.setInverted(driveEncoderReversed);
        turningMotor.setInverted(turningEncoderReversed);

        turningPID.enableContinuousInput(-Math.PI, Math.PI);
    }  
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(driveMotor.getSelectedSensorPosition()*ModuleConstants.TICperIn, 
                new Rotation2d(turningMotor.getSelectedSensorPosition()*ModuleConstants.TICperIn));
    }
    public void setDesiredState(SwerveModuleState state) {
        // Calculate the drive output from the drive PID controller.
        final var driveOutput = drivePID.calculate(
            driveMotor.getSelectedSensorPosition()*ModuleConstants.TICperIn, state.speedMetersPerSecond);
    
        // Calculate the turning motor output from the turning PID controller.
        final var turnOutput = turningPID.calculate(
            turningMotor.getSelectedSensorPosition()*ModuleConstants.TICperIn, state.angle.getRadians()
        );
    
        // Calculate the turning motor output from the turning PID controller.
        driveMotor.set(ControlMode.PercentOutput, driveOutput);
        turningMotor.set(ControlMode.PercentOutput, turnOutput);
      }
      public void resetEncoders(){
          driveMotor.setSelectedSensorPosition(0);
          turningMotor.setSelectedSensorPosition(0);
      }
    
}

