package frc.robot;

import com.techhounds.houndutil.houndlib.subsystems.BaseDifferentialDrive.DifferentialDriveMode;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.Arm.ArmPosition;
import frc.robot.Constants.Elevator.ElevatorPosition;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class Controls {
    public static void configureControls(int port, Drivetrain drivetrain, Elevator elevator, Arm arm, Intake intake,
            Climber climber) {

        CommandXboxController controller = new CommandXboxController(port);

        drivetrain.setDefaultCommand(
                drivetrain.teleopDriveCommand(
                        () -> -controller.getLeftY(),
                        () -> -controller.getRightY(),
                        () -> -controller.getRightX(),
                        () -> DifferentialDriveMode.ARCADE));

        controller.x().whileTrue(elevator.moveToPositionCommand(() -> ElevatorPosition.L2)
                .alongWith(arm.moveToPositionCommand(() -> ArmPosition.SCORING)));
        controller.y().whileTrue(elevator.moveToPositionCommand(() -> ElevatorPosition.L4)
                .alongWith(arm.moveToPositionCommand(() -> ArmPosition.SCORING)));
        controller.a().whileTrue(elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM)
                .alongWith(arm.moveToPositionCommand(() -> ArmPosition.TOP)));
        controller.b().whileTrue(elevator.moveToPositionCommand(() -> ElevatorPosition.INTAKE)
                .alongWith(arm.moveToPositionCommand(() -> ArmPosition.BOTTOM)));

        controller.povDown().onTrue(RobotCommands.scoreCoralCommand(elevator, arm));

        // controller.x().and(controller.povUp()).whileTrue(elevator.moveToPositionCommand(()
        // -> ElevatorPosition.TOP));
        // controller.x().and(controller.povLeft()).whileTrue(elevator.moveToPositionCommand(()
        // -> ElevatorPosition.L2));
        // controller.x().and(controller.povRight()).whileTrue(elevator.moveToPositionCommand(()
        // -> ElevatorPosition.L3));
        // controller.x().and(controller.povDown())
        // .whileTrue(elevator.moveToPositionCommand(() -> ElevatorPosition.BOTTOM));

        // controller.y().and(controller.povUp()).whileTrue(arm.moveToPositionCommand(()
        // -> ArmPosition.TOP));
        // controller.y().and(controller.povLeft()).whileTrue(arm.moveToPositionCommand(()
        // -> ArmPosition.HORIZONTAL));
        // controller.y().and(controller.povRight()).whileTrue(arm.moveToPositionCommand(()
        // -> ArmPosition.SCORING));
        // controller.y().and(controller.povDown()).whileTrue(arm.moveToPositionCommand(()
        // -> ArmPosition.BOTTOM));

        // controller.povUp().and(controller.x().whileTrue(elevator.sysIdQuasistaticCommand(Direction.kForward)));
        // controller.povUp().and(controller.y().whileTrue(elevator.sysIdQuasistaticCommand(Direction.kReverse)));
        // controller.povUp().and(controller.a().whileTrue(elevator.sysIdDynamicCommand(Direction.kForward)));
        // controller.povUp().and(controller.b().whileTrue(elevator.sysIdDynamicCommand(Direction.kReverse)));

        // controller.povRight().and(controller.x().whileTrue(arm.sysIdQuasistaticCommand(Direction.kForward)));
        // controller.povRight().and(controller.y().whileTrue(arm.sysIdQuasistaticCommand(Direction.kReverse)));
        // controller.povRight().and(controller.a().whileTrue(arm.sysIdDynamicCommand(Direction.kForward)));
        // controller.povRight().and(controller.b().whileTrue(arm.sysIdDynamicCommand(Direction.kReverse)));
        // controller.povLeft().onTrue(drivetrain.resetGyroCommand().ignoringDisable(true));

    }

    public static void configureTestingControls(int port, Drivetrain drivetrain, Elevator elevator, Arm arm,
            Intake intake,
            Climber climber) {

        CommandXboxController controller = new CommandXboxController(port);
        elevator.setDefaultCommand(elevator.setOverridenSpeedCommand(() -> -controller.getLeftY()));
        arm.setDefaultCommand(arm.setOverridenSpeedCommand(() -> -controller.getRightY()));

    }
}
