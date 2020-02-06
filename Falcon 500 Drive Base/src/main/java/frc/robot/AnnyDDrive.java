import java.util.StringJoiner;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpiutil.math.MathUtil;

public class AnnyDDrive extends RobotDriveBase implements Sendable, AutoCloseable {
    public static final double kDefaultQuickStopThreshold = 0.2;
    public static final double kDefaultQuickStopAlpha = 0.1;
  
    private static int instances;
  
    private final TalonFX m_FR;
    private final TalonFX m_BR;
    private final TalonFX m_FL;
    private final TalonFX m_BL;
  
    private double m_quickStopThreshold = kDefaultQuickStopThreshold;
    private double m_quickStopAlpha = kDefaultQuickStopAlpha;
    private double m_quickStopAccumulator;
    private double m_rightSideInvertMultiplier = -1.0;
    private boolean m_reported;

    public AnnyDDrive(TalonFX FR, TalonFX BR, TalonFX FL, TalonFX BL) {
        verify(FR, BR, FL, BL);
        m_FR = FR;
        m_BR = BR;
        m_FL = FL;
        m_BL = BL;
        SendableRegistry.addChild(this, FR);
        SendableRegistry.addChild(this, BR);
        SendableRegistry.addChild(this, FL);
        SendableRegistry.addChild(this, BL);
        m_FR.follow(BR);
        m_FL.follow(BL);
        instances++;
        SendableRegistry.addLW(this, "AnnyDDrive", instances);
    }



    @SuppressWarnings("PMD.AvoidThrowingNullPointerException")
    private void verify(TalonFX FR, TalonFX BR, TalonFX FL, TalonFX BL) {
        if (FR != null && BR != null && FL != null && BL != null)
            return; 
        StringJoiner joiner = new StringJoiner(", ");
        if (FR == null) {
            joiner.add("FR");
        }
        if (BR == null) {
            joiner.add("BR");
        }
        if (FL == null) {
            joiner.add("FL");
        }
        if (BL == null) {
            joiner.add("BL");
        }
        throw new NullPointerException(joiner.toString());  
    }
    @Override
    public void close() throws Exception {
        SendableRegistry.remove(this);
    }
    @SuppressWarnings("ParameterName")
    public void arcadeDrive(double xSpeed, double zRotation) {
        arcadeDrive(xSpeed, zRotation, true);
    }
    @SuppressWarnings("ParameterName")
    public void arcadeDrive(double xSpeed, double zRotation, boolean squareInputs) {
        if (!m_reported) {
        HAL.report(tResourceType.kResourceType_RobotDrive,
                    tInstances.kRobotDrive2_DifferentialArcade, 2);
        m_reported = true;
        }

        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        xSpeed = applyDeadband(xSpeed, m_deadband);

        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);
        zRotation = applyDeadband(zRotation, m_deadband);

        // Square the inputs (while preserving the sign) to increase fine control
        // while permitting full power.
        if (squareInputs) {
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        zRotation = Math.copySign(zRotation * zRotation, zRotation);
        }

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);

        if (xSpeed >= 0.0) {
        // First quadrant, else second quadrant
        if (zRotation >= 0.0) {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
        } else {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
        }
        } else {
        // Third quadrant, else fourth quadrant
        if (zRotation >= 0.0) {
            leftMotorOutput = xSpeed + zRotation;
            rightMotorOutput = maxInput;
        } else {
            leftMotorOutput = maxInput;
            rightMotorOutput = xSpeed - zRotation;
        }
        }

        m_BL.set(ControlMode.PercentOutput, MathUtil.clamp(leftMotorOutput, -1.0, 1.0) * m_maxOutput);
        double maxOutput = m_maxOutput * m_rightSideInvertMultiplier;
        m_BL.set(ControlMode.PercentOutput, MathUtil.clamp(rightMotorOutput, -1.0, 1.0) * maxOutput);

        feed();
    }
    @Override
    public void initSendable(SendableBuilder builder) {
        // TODO Auto-generated method stub

    }

    @Override
    public void stopMotor() {
        m_BL.set(ControlMode.PercentOutput, 0);
        m_BR.set(ControlMode.PercentOutput, 0);
        m_FL.set(ControlMode.PercentOutput, 0);
        m_FR.set(ControlMode.PercentOutput, 0);
        feed();
    }

    @Override
    public String getDescription() {
        return "Anny's DifferentialDrive to work with TalonFX";
    }
}