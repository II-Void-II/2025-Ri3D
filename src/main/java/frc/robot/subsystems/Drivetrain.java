package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.commands.FollowPathRamsete;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlib.SparkConfigurator;
import com.techhounds.houndutil.houndlog.interfaces.Log;
import com.techhounds.houndutil.houndlog.interfaces.LoggedObject;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.Controls.*;
import static frc.robot.Constants.LOOP_TIME;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@LoggedObject
public class Drivetrain extends SubsystemBase {
    @Log
    private final CANSparkMax leftPrimaryMotor;
    @Log
    private final CANSparkMax leftSecondaryMotor;
    @Log
    private final CANSparkMax rightPrimaryMotor;
    @Log
    private final CANSparkMax rightSecondaryMotor;

    @Log
    private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);

    private final DifferentialDrivePoseEstimator poseEstimator;

    private final MutableMeasure<Voltage> sysidAppliedVoltageMeasure = MutableMeasure.mutable(Volts.of(0));
    private final MutableMeasure<Distance> sysidPositionMeasure = MutableMeasure.mutable(Meters.of(0));
    private final MutableMeasure<Velocity<Distance>> sysidVelocityMeasure = MutableMeasure
            .mutable(MetersPerSecond.of(0));

    private final SysIdRoutine sysIdRoutine;

    @Log
    private double leftFeedbackVoltage;
    @Log
    private double rightFeedbackVoltage;
    @Log
    private double leftFeedforwardVoltage;
    @Log
    private double rightFeedforwardVoltage;

    @Log
    private final ProfiledPIDController leftPidController = new ProfiledPIDController(kP, kI, kD,
            XY_CONSTRAINTS);
    @Log
    private final ProfiledPIDController rightPidController = new ProfiledPIDController(kP, kI, kD,
            XY_CONSTRAINTS);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV_LINEAR, kA_LINEAR);

    public Drivetrain() {
        leftPrimaryMotor = SparkConfigurator.createSparkMax(LEFT_PRIMARY_MOTOR_ID, MotorType.kBrushless,
                LEFT_DRIVE_MOTORS_INVERTED,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));
        leftSecondaryMotor = SparkConfigurator.createSparkMax(LEFT_SECONDARY_MOTOR_ID, MotorType.kBrushless,
                LEFT_DRIVE_MOTORS_INVERTED,
                (s) -> s.follow(leftPrimaryMotor),
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));

        rightPrimaryMotor = SparkConfigurator.createSparkMax(RIGHT_PRIMARY_MOTOR_ID, MotorType.kBrushless,
                RIGHT_DRIVE_MOTORS_INVERTED,
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));

        rightSecondaryMotor = SparkConfigurator.createSparkMax(RIGHT_SECONDARY_MOTOR_ID, MotorType.kBrushless,
                RIGHT_DRIVE_MOTORS_INVERTED,
                (s) -> s.follow(rightPrimaryMotor),
                (s) -> s.setIdleMode(IdleMode.kBrake),
                (s) -> s.setSmartCurrentLimit(CURRENT_LIMIT),
                (s) -> s.getEncoder().setPositionConversionFactor(ENCODER_ROTATIONS_TO_METERS),
                (s) -> s.getEncoder().setVelocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0));

        poseEstimator = new DifferentialDrivePoseEstimator(
                KINEMATICS, getRotation(), getWheelPositions().leftMeters, getWheelPositions().rightMeters,
                new Pose2d());

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(1).per(Seconds.of(1)), Volts.of(8), null, null),
                new SysIdRoutine.Mechanism(
                        (Measure<Voltage> volts) -> setVoltage(volts.magnitude()),
                        log -> {
                            log.motor("primary")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(
                                            (RobotBase.isSimulation() ? 1 : 12) * leftPrimaryMotor.getAppliedOutput(),
                                            Volts))
                                    .linearPosition(sysidPositionMeasure
                                            .mut_replace(leftPrimaryMotor.getEncoder().getPosition(), Meters))
                                    .linearVelocity(sysidVelocityMeasure
                                            .mut_replace(leftPrimaryMotor.getEncoder().getVelocity(), MetersPerSecond));
                        },
                        this));

        AutoManager.getInstance().setResetOdometryConsumer(this::resetPoseEstimator);
    }

    public void periodic() {
        updatePoseEstimator();
        drawRobotOnField(AutoManager.getInstance().getField());
    }

    private void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());
    }

    public void simulationPeriodic() {

    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return gyro.getRotation2d();
    }

    @Log
    public DifferentialDriveWheelPositions getWheelPositions() {
        return new DifferentialDriveWheelPositions(
                leftPrimaryMotor.getEncoder().getPosition(),
                rightPrimaryMotor.getEncoder().getPosition());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                leftPrimaryMotor.getEncoder().getVelocity(),
                rightPrimaryMotor.getEncoder().getVelocity());
    }

    public DifferentialDriveWheelVoltages getWheelVoltages() {
        return new DifferentialDriveWheelVoltages(
                leftPrimaryMotor.getAppliedOutput(),
                rightPrimaryMotor.getAppliedOutput());
    }

    public ChassisSpeeds getChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getWheelSpeeds());
    }

    public DifferentialDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    public void updatePoseEstimator() {
        poseEstimator.update(getRotation(), getWheelPositions());
    }

    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(), getWheelPositions(), pose);
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void setMotorHoldModes(MotorHoldMode motorHoldMode) {
        if (motorHoldMode == MotorHoldMode.COAST) {
            leftPrimaryMotor.setIdleMode(IdleMode.kCoast);
            leftSecondaryMotor.setIdleMode(IdleMode.kCoast);
            rightPrimaryMotor.setIdleMode(IdleMode.kCoast);
            rightSecondaryMotor.setIdleMode(IdleMode.kCoast);
        } else {
            leftPrimaryMotor.setIdleMode(IdleMode.kBrake);
            leftSecondaryMotor.setIdleMode(IdleMode.kBrake);
            rightPrimaryMotor.setIdleMode(IdleMode.kBrake);
            rightSecondaryMotor.setIdleMode(IdleMode.kBrake);
        }
    }

    public void setCurrentLimit(int currentLimit) {
        leftPrimaryMotor.setSmartCurrentLimit(currentLimit);
        leftSecondaryMotor.setSmartCurrentLimit(currentLimit);
        rightPrimaryMotor.setSmartCurrentLimit(currentLimit);
        rightSecondaryMotor.setSmartCurrentLimit(currentLimit);
    }

    public void stop() {
        leftPrimaryMotor.stopMotor();
        leftSecondaryMotor.stopMotor();
        rightPrimaryMotor.stopMotor();
        rightSecondaryMotor.stopMotor();
    }

    private void setVoltage(double volts) {
        leftPrimaryMotor.setVoltage(volts);
        rightPrimaryMotor.setVoltage(volts);
    }

    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = KINEMATICS.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        leftPrimaryMotor.setVoltage((wheelSpeeds.leftMetersPerSecond / MAX_DRIVING_VELOCITY_METERS_PER_SECOND) * 12.0);
        rightPrimaryMotor
                .setVoltage((wheelSpeeds.rightMetersPerSecond / MAX_DRIVING_VELOCITY_METERS_PER_SECOND) * 12.0);
    }

    public void driveClosedLoop(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = KINEMATICS.toWheelSpeeds(speeds);
        DifferentialDriveWheelSpeeds currentWheelSpeeds = getWheelSpeeds();
        wheelSpeeds.desaturate(MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        leftFeedbackVoltage = leftPidController.calculate(currentWheelSpeeds.leftMetersPerSecond,
                wheelSpeeds.leftMetersPerSecond);
        rightFeedbackVoltage = rightPidController.calculate(currentWheelSpeeds.rightMetersPerSecond,
                wheelSpeeds.rightMetersPerSecond);

        leftFeedforwardVoltage = feedforward.calculate(
                wheelSpeeds.leftMetersPerSecond);
        rightFeedforwardVoltage = feedforward.calculate(
                wheelSpeeds.rightMetersPerSecond);

        leftPrimaryMotor.setVoltage(leftFeedbackVoltage + leftFeedforwardVoltage);
        rightPrimaryMotor.setVoltage(rightFeedbackVoltage + rightFeedforwardVoltage);
    }

    public Command teleopDriveCommand(DoubleSupplier leftStickThrustSupplier, DoubleSupplier rightStickThrustSupplier,
            DoubleSupplier rightStickRotationSupplier,
            Supplier<DifferentialDriveMode> driveModeSupplier) {
        SlewRateLimiter leftThrustLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter rightThrustLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter rotationLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);

        return run(() -> {
            double leftThrust = leftStickThrustSupplier.getAsDouble();
            double rightThrust = rightStickThrustSupplier.getAsDouble();
            double rightRotation = rightStickRotationSupplier.getAsDouble();
            DifferentialDriveMode driveMode = driveModeSupplier.get();

            leftThrust = Math.copySign(Math.pow(leftThrust, JOYSTICK_CURVE_EXP), leftThrust);
            rightThrust = Math.copySign(Math.pow(rightThrust, JOYSTICK_CURVE_EXP), rightThrust);
            rightRotation = Math.copySign(Math.pow(rightRotation, JOYSTICK_ROT_CURVE_EXP), rightRotation);

            leftThrust = leftThrustLimiter.calculate(leftThrust);
            rightThrust = rightThrustLimiter.calculate(rightThrust);
            rightRotation = rotationLimiter.calculate(rightRotation);

            leftThrust *= MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            rightThrust *= MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
            rightRotation *= MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

            switch (driveMode) {
                case ARCADE -> {
                    drive(new ChassisSpeeds(leftThrust, 0, rightRotation));
                }
                case TANK -> {
                    // the speeds are initially values from -1.0 to 1.0, so we multiply by the max
                    // physical velocity to output in m/s.
                    ChassisSpeeds speeds = KINEMATICS.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
                            leftThrust, rightThrust));

                    drive(speeds);
                }
            }
        }).withName("drivetrain.teleopDrive");

    }

    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathRamsete(
                path,
                this::getPose, // Robot pose supplier
                this::getChassisSpeeds, // Current ChassisSpeeds supplier
                this::drive, // Method that will drive the robot given ChassisSpeeds
                new ReplanningConfig(), // Default path replanning config. See the API for the options here
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this // Reference to this subsystem to set requirements
        );
    }

    public Command resetGyroCommand() {
        return Commands.runOnce(() -> this.resetGyro()).withName("drivetrain.resetGyro");
    }

    public Command setCurrentLimitCommand(int currentLimit) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCurrentLimitCommand'");
    }

    public Command coastMotorsCommand() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'coastMotorsCommand'");
    }

    public Command sysidQuasistaticForwardCommand() {
        return sysIdRoutine.quasistatic(Direction.kForward);
    }

    public Command sysidQuasistaticReverseCommand() {
        return sysIdRoutine.quasistatic(Direction.kReverse);
    }

    public Command sysidDynamicForwardCommand() {
        return sysIdRoutine.dynamic(Direction.kForward);
    }

    public Command sysidDynamicReverseCommand() {
        return sysIdRoutine.dynamic(Direction.kReverse);
    }
}
