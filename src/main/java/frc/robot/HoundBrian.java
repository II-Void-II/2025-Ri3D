package frc.robot;

import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

@LoggedObject
public class HoundBrian {
    @Log
    private final DigitalInput drivetrainButton = new DigitalInput(0);
    @Log
    private final DigitalInput elevatorButton = new DigitalInput(1);
    @Log
    private final DigitalInput armButton = new DigitalInput(2);
    @Log
    private final DigitalInput climberButton = new DigitalInput(3);
    @Log
    private final DigitalInput unusedButton2 = new DigitalInput(4);
    @Log
    private final DigitalInput unusedButton3 = new DigitalInput(5);
    @Log
    private final DigitalInput unusedButton4 = new DigitalInput(6);

    // private final DIOSim drivetrainButtonSim = new DIOSim(drivetrainButton);
    // private final DIOSim intakeButtonSim = new DIOSim(intakeButton);
    // private final DIOSim shooterTiltButtonSim = new DIOSim(shooterTiltButton);
    // private final DIOSim climberButtonSim = new DIOSim(climberButton);
    // private final DIOSim noteLiftButtonSim = new DIOSim(noteLiftButton);
    // private final DIOSim actionButtonSim = new DIOSim(actionButton);
    // private final DIOSim actionButton2Sim = new DIOSim(actionButton2);

    public HoundBrian(Drivetrain drivetrain, Elevator elevator, Arm arm, Climber climber) {

        new Trigger(drivetrainButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(drivetrain.resetGyroCommand().ignoringDisable(true));
        new Trigger(elevatorButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(elevator.resetPositionCommand().ignoringDisable(true));
        new Trigger(armButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(arm.resetPositionCommand().ignoringDisable(true));
        new Trigger(climberButton::get).negate()
                .and(DriverStation::isDisabled)
                .onTrue(climber.resetPositionCommand().ignoringDisable(true));

        // new Trigger(intakeButton::get).debounce(3, DebounceType.kBoth)
        // .and(DriverStation::isDisabled)
        // .toggleOnTrue(intake.coastMotorsCommand());
        // new Trigger(shooterTiltButton::get).debounce(3, DebounceType.kBoth)
        // .and(DriverStation::isDisabled)
        // .toggleOnTrue(shooterTilt.coastMotorsCommand());
        // new Trigger(climberButton::get).debounce(3, DebounceType.kBoth)
        // .and(DriverStation::isDisabled)
        // .toggleOnTrue(climber.coastMotorsCommand());
        // new Trigger(noteLiftButton::get).debounce(3, DebounceType.kBoth)
        // .and(DriverStation::isDisabled)
        // .toggleOnTrue(noteLift.coastMotorsCommand());

        // if (RobotBase.isSimulation()) {
        // drivetrainButtonSim.setValue(true);
        // intakeButtonSim.setValue(true);
        // shooterTiltButtonSim.setValue(true);
        // climberButtonSim.setValue(true);
        // noteLiftButtonSim.setValue(true);
        // actionButtonSim.setValue(true);
        // actionButton2Sim.setValue(true);
        // }
    }

    // public Command simTriggerIntakeButton() {
    // return Commands.startEnd(() -> intakeButtonSim.setValue(false), () ->
    // intakeButtonSim.setValue(true))
    // .ignoringDisable(true);
    // }

}
