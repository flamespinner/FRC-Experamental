package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class LimeLightSearch extends SequentialCommandGroup{
    public static boolean PathToTake;

    public LimeLightSearch() {
        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        //boolean PathToTake;
        if (tv <1) {
            PathToTake = false;
        } else {
            PathToTake = true;
        }

        SmartDashboard.putBoolean("limelight vision", PathToTake);
    }
}
