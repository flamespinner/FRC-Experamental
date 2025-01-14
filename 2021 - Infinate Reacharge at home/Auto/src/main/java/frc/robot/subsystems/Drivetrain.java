package frc.robot.subsystems;

/*IMPORTANT NOTES
    EVERYTHING NEEDS TO BE IN SI UNITS/METERS

*/

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
/*import com.ctre.phoenix.motorcontrol.FeedbackDevice;

//TalonFX/Falcon500 Imports
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants.DriveConstants;*/

public class Drivetrain extends SubsystemBase {
    /*private final WPI_TalonSRX leftFront = new WPI_TalonSRX(1);
    private final WPI_TalonSRX leftBack = new WPI_TalonSRX(2);
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(3);
    private final WPI_TalonSRX rightBack = new WPI_TalonSRX(4);*/

    private final WPI_TalonFX falconBL = new WPI_TalonFX(DriveConstants.FALCON_BL_ID); 
    private final WPI_TalonFX falconFL = new WPI_TalonFX(DriveConstants.FALCON_FL_ID); 
    private final WPI_TalonFX falconBR = new WPI_TalonFX(DriveConstants.FALCON_BR_ID); 
    private final WPI_TalonFX falconFR = new WPI_TalonFX(DriveConstants.FALCON_FR_ID); 

  
    //private final TalonFXConfiguration fxConfig = new TalonFXConfiguration();

    AHRS gyro = new AHRS(SPI.Port.kMXP); //define Gyro Port Gyro Type

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(AutoConstants.kTrackwidthMeters);

    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading()); //TODO Verify this works ok


    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltsSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter); //ks, kv, ka values from the frc-characterization tool

    PIDController leftPidController = new PIDController(AutoConstants.kPDriveVel, 0, 0); 
    PIDController rightPidController = new PIDController(AutoConstants.kPDriveVel, 0, 0); 

    Pose2d pose; //contains x, y and heading position data

    public Drivetrain() {
        falconBL.follow(falconFL);
        falconBR.follow(falconFR);

        falconFR.setInverted(true);
        falconBR.setInverted(InvertType.FollowMaster);
        falconFL.setInverted(false);
        falconBL.setInverted(InvertType.FollowMaster);/*

        fxConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;*/
        //falconBR.setInverted(true);
        //falconBL.setInverted(false);

        /*leftBack.follow(leftFront);
        rightBack.follow(rightFront);

        rightBack.setInverted(true);
        leftBack.setInverted(false);*/
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle());
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds( //Math to go from RPM to meteres per second (SI Units)
            //falconFR.getSelectedSensorVelocity() / 10.75 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60,
            //falconFL.getSelectedSensorVelocity() / 10.75 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60

            falconFL.getSelectedSensorVelocity() / 10.75 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60, 
            falconFR.getSelectedSensorVelocity() / 10.75 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60

        );
    }

    public SimpleMotorFeedforward getFeedForward() {
        return feedforward;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public PIDController getLeftPIDController() {
        return leftPidController;
    }

    public PIDController getRightPIDController() { 
        return rightPidController;
    }

    public Pose2d getPose() {
        return pose;
    }

    public void setOutputVolts(double leftVolts, double rightVolts) {
        falconFR.setVoltage(leftVolts / 12);
        falconFR.setVoltage(rightVolts / 12);
        //leftFront.setVoltage(leftVolts / 12);
        //rightFront.setVoltage(rightVolts / 12);
    }

    public void reset() {
        odometry.resetPosition(new Pose2d(), getHeading());
    }

    @Override
    public void periodic() { //runs periodicly (every 20ms)
    //    pose = odometry.update(getHeading(), getSpeeds());
    
    //GyroAngle - Angle Reported from the Gyro
    //Encoder.getDistance() - Distance Travelled by the left encoder    
    pose = odometry.update(getHeading(), 
                            falconFL.getSelectedSensorPosition(), 
                            falconFR.getSelectedSensorPosition()); //TODO VERIFY
    }

}


//FRC-Characterization tool 40:00
//PID Information in FRC C. Tool 45:00
//Pathweaver 49:30
//58.39