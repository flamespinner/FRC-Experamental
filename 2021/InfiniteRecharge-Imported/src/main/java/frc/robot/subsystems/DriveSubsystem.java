package frc.robot.subsystems;

//Motor Imports
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

//Sensor Imports
import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

//Auto Imports
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

//Util Imports
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;


/**
 * this subsystem sets up and directly manipulates everything on the drive train
 */
public class DriveSubsystem extends SubsystemBase {
    
    //Declare and intializing Falcon500s
    //Left Drive Train Motors
    private final WPI_TalonFX falconFL = new WPI_TalonFX(DriveConstants.FALCON_FL_ID);
    private final WPI_TalonFX falconBL = new WPI_TalonFX(DriveConstants.FALCON_BL_ID);

    //Right Drive train Motors
    private final WPI_TalonFX falconFR = new WPI_TalonFX(DriveConstants.FALCON_FR_ID);
    private final WPI_TalonFX falconBR = new WPI_TalonFX(DriveConstants.FALCON_BR_ID);

    DifferentialDriveKinematics Kinematics =
            new DifferentialDriveKinematics(AutoConstants.kTrackwidthMeters);

    //Creating Speed Controller Groups
    //Motors on the right side of the drive
    private final SpeedControllerGroup SCG_R = new SpeedControllerGroup(falconFR, falconBR);

    //Motors on the left side of the drive
    private final SpeedControllerGroup SCG_L = new SpeedControllerGroup(falconFL, falconBL);

    //The Robots Drive
    private final DifferentialDrive drive = new DifferentialDrive(SCG_L, SCG_R);

    //Define new Navx Gyro on MXP
    AHRS gyro = new AHRS(SPI.Port.kMXP);

    //Odometry class for tracking robot pose
   private final DifferentialDriveOdometry odometry;

   private double error;
   

    /**
     * Creates a new DriveSubsystem
     */
    public DriveSubsystem() {
        setBrake();
        resetEncoders();
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

        //set followers
        falconBR.follow(falconFR);
        falconBL.follow(falconFL);
    }

    /**
     * Sets the Falcon500s to break mode.
     */
    public void setBrake() {
        falconBL.setNeutralMode(NeutralMode.Brake);
        falconFL.setNeutralMode(NeutralMode.Brake);
        falconBR.setNeutralMode(NeutralMode.Brake);
        falconFR.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        //Update the odometry in the periodic block
        odometry.update(gyro.getRotation2d(), falconFL.getSelectedSensorPosition(), falconFR.getSelectedSensorPosition());
    }

    /**
     * Returns the currently-estimated pose of the robot
     * 
     * @return The Pose
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot
     * 
     * @return The Current wheel speeds
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(falconFL.getSelectedSensorVelocity() / 10.71 * 2 * Units.inchesToMeters(3.0) / 60, falconFR.getSelectedSensorVelocity() / 10.71 *2 * Units.inchesToMeters(3) / 60);
    }

    /**
     * Resets the odometry to the specified pose
     * 
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdeometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

    /**
     * Drives the robot using arcade controls
     * 
     * @param fwd the command forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) { //TODO Change
        drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages
     * 
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        SCG_L.setVoltage(leftVolts);
        SCG_R.setVoltage(rightVolts);
        drive.feed();
    }
    
    /**
     * Resets the drive encoders to currently read a position of 0.
     */
     public void resetEncoders() {
        falconFR.setSelectedSensorPosition(0);
        falconFL.setSelectedSensorPosition(0);
        falconBR.setSelectedSensorPosition(0);
        falconBL.setSelectedSensorPosition(0);
    }

    /**
    * Gets the average distance of the two encoders
    *
    *@return the average of the two encoder readings
    */
    public double getAverageEncoderDistance() {
        return (falconFL.getSelectedSensorPosition() + falconFR.getSelectedSensorPosition()) / 2.0; //TODO VERIFY
    }

    /**
     * Gets the left drive encoder
     * 
     * @return the left drive encoder
     */
    /*public Encoder getLeftEncoder() { //TODO FIX
        return leftEncoder;
    }*/

    /**
     * Gets the right drive encoder
     * 
     * @return the right drive encoder
     */
    /*public Encoder getRightEncoder() {
        return rightEncoder;
    }*/

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     * 
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot
     * 
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot
     * 
     * @return the turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -gyro.getRate();
    }

    /**
     * Returns Kinematics
     * 
     * @return the robots Kinematics
     */
    public DifferentialDriveKinematics getKinematics() {
        return Kinematics;
    }

  /**
   * set the speed of the drive train with tank controls (WIP)
   * @param lSpeed
   * @param rSpeed
   */
  public void setDriveSpeed_Tank(double lSpeed, double rSpeed) {
    drive.tankDrive(lSpeed, rSpeed);
  }

/**
 * sets the speed of the drive train with arcade controls
 * @param xSpeed
 * @param zRotation
 */
  public void setDriveSpeed_Arcade(double xSpeed, double zRotation) {
    if (zRotation == 0 )
        driveStraight(xSpeed);
    drive.arcadeDrive(xSpeed, zRotation);
  }

  private double driveTrainP() {
    error = falconFL.getSelectedSensorPosition() - falconFR.getSelectedSensorPosition();
    //integral += error*.02;
    return DriveConstants.DRIVE_P*error;
  }

  public void driveStraight(double xSpeed) {
    drive.arcadeDrive(xSpeed, -driveTrainP());
  }

}
