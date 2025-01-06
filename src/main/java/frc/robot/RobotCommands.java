package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class RobotCommands {
    public static Command scoreCoralCommand(Elevator elevator, Arm arm) {
        return Commands.parallel(
                elevator.movePositionDeltaCommand(() -> Constants.Elevator.SCORING_MOVEMENT),
                Commands.waitSeconds(0.15)
                        .andThen(arm.movePositionDeltaCommand(() -> Constants.Elevator.SCORING_MOVEMENT)));
    }
}
