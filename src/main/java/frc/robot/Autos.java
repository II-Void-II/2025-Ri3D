package frc.robot;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.techhounds.houndutil.houndauto.AutoRoutine;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;

public class Autos {
    public static AutoRoutine testPath(Drivetrain drivetrain) {
        PathPlannerPath path = PathPlannerPath.fromPathFile("testPath");
        Command command = drivetrain.followPathCommand(path);
        return new AutoRoutine("testPath", command, List.of(path), path.getStartingDifferentialPose());
    }

    public static AutoRoutine GDC(Drivetrain drivetrain) {
        PathPlannerPath Start_GPath = PathPlannerPath.fromPathFile("Start-G");
        PathPlannerPath G_PickupPath = PathPlannerPath.fromPathFile("G-Pickup");
        PathPlannerPath Pickup_DPath = PathPlannerPath.fromPathFile("Pickup-D");
        PathPlannerPath D_PickupPath = PathPlannerPath.fromPathFile("D-Pickup");
        PathPlannerPath Pickup_CPath = PathPlannerPath.fromPathFile("Pickup-C");

        Command command = Commands.sequence(
                drivetrain.followPathCommand(Start_GPath),
                Commands.waitSeconds(1),
                drivetrain.followPathCommand(G_PickupPath),
                Commands.waitSeconds(1),
                drivetrain.followPathCommand(Pickup_DPath),
                Commands.waitSeconds(1),
                drivetrain.followPathCommand(D_PickupPath),
                Commands.waitSeconds(1),
                drivetrain.followPathCommand(Pickup_CPath));

        return new AutoRoutine("GDC", command,
                List.of(Start_GPath, G_PickupPath, Pickup_DPath, D_PickupPath, Pickup_CPath),
                Start_GPath.getStartingDifferentialPose());
    }
}
