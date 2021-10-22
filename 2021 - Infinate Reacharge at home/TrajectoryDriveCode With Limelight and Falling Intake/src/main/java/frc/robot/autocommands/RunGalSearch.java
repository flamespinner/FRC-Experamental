package frc.robot.autocommands;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCross;
import frc.robot.commands.LimeLightSearch;
import frc.robot.commands.RunPath;
import frc.robot.subsystems.DriveTrain;

public class RunGalSearch extends SequentialCommandGroup {
    private SequentialCommandGroup search;


    public RunGalSearch(DriveTrain drive) {
        search = new SequentialCommandGroup(new AutoCross(drive),
                                                new ParallelRaceGroup(
                                                    new LimeLightSearch()
                                                ),
                                            new RunPath(drive)
                                            );
    }
}
