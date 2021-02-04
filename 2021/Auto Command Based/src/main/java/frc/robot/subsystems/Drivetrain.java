package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.AnalogGyro;
import com.analog.adis16448.frc.ADIS16448_IMU;
import edu.wpi.first.wpilibj2.Units;


public class Drivetrain extends SubsystemBase {

    private final WPI_TalonSRX leftFront = new WPI_TalonSRX(1); // TODO ID
    private final WPI_TalonSRX leftBack = new WPI_TalonSRX(2); // TODO ID
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(3); // TODO ID
    private final WPI_TalonSRX rightBack = new WPI_TalonSRX(4); // TODO ID

    public static final ADIS16448_IMU gyro = new ADIS16448_IMU();

    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27.0)); //Creating Kinematics object: track width of 27 inches
    DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(kinematics, getHeading());

    SimpleMotorFeedForward feedforward = new SimpleMotorFeedForward(0.268, 1.89, 0.243);

    Pose2d pose;

    public Drivetrain() {
        leftBack.follow(leftFront);
        rightBack.follow(rightFront);

        leftBack.setInverted(false);
        rightBack.setInverted(true);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(-gyro.getAngle()); 
    }

    public DifferentialDriveWheelSpeeds getSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftMaster.getEncoder().getVelocity() / 10.71 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60, //devided by gear ratio  -- wheel speeds meters/s
            rightMaster.getEncoder().getVelocity() / 10.71 * 2 * Math.PI * Units.inchesToMeters(3.0) / 60 //devided by gear ratio -- wheel speeds meters/s
        );
    }

    @Override
    public void periodic() { //happens every 20ms
//        pose = odometry.update(getHeading(), getSpeeds());

    }
}


//https://youtu.be/wqJ4tY0u6IQ?t=2561