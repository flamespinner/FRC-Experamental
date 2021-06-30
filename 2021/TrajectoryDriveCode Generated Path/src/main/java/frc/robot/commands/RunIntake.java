package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveTrain;

public class RunIntake extends CommandBase{
    private static DriveTrain m_driveSub;

    public RunIntake(DriveTrain driveSub) {
        m_driveSub = driveSub;

        addRequirements(m_driveSub);
    }

    @Override
    public void initialize() {
        WPI_TalonFX leftFront = new WPI_TalonFX(DriveConstants.LeftFront);
        while (leftFront.getSelectedSensorPosition() < 2500) {
            m_driveSub.setDriveSpeed_Arcade(1,0);
        }
        while (leftFront.getSelectedSensorPosition() > 0) {
            m_driveSub.setDriveSpeed_Arcade(-1,0);
        }
    }

    @Override
    public void execute() {
        m_driveSub.setDriveSpeed_Arcade(.5,0);
    }

    @Override
    public void end(boolean interrupted) {
        m_driveSub.setDriveSpeed_Arcade(0, 0);
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }
    
}
