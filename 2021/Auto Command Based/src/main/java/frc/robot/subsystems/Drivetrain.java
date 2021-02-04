package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Drivetrain extends SubsystemBase {

    private final WPI_TalonSRX leftFront = new WPI_TalonSRX(1); // TODO ID
    private final WPI_TalonSRX leftBack = new WPI_TalonSRX(2); // TODO ID
    private final WPI_TalonSRX rightFront = new WPI_TalonSRX(3); // TODO ID
    private final WPI_TalonSRX rightBack = new WPI_TalonSRX(4); // TODO ID

    public Drivetrain() {
        leftBack.follow(leftFront);
        rightBack.follow(rightFront);

        leftBack.setInverted(false);
        rightBack.setInverted(true);
    }

}
