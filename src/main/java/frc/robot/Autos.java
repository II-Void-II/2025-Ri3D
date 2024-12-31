package frc.robot;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndauto.AutoRoutine;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class Autos {
    public static AutoRoutine testPath(Drivetrain drivetrain) {
        PathPlannerPath path = PathPlannerPath.fromPathFile("testPath");
        Command command = drivetrain.followPathCommand(path);
        return new AutoRoutine("testPath", command, List.of(path), path.getStartingDifferentialPose());
    }
}
