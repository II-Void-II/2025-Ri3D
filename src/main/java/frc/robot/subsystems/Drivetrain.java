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
import com.techhounds.houndutil.houndlib.subsystems.BaseDifferentialDrive;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.LOOP_TIME;
import static frc.robot.Constants.Controls.*;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@LoggedObject
public class Drivetrain extends SubsystemBase implements BaseDifferentialDrive {
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
    private DifferentialDriveWheelVoltages feedbackVoltages = new DifferentialDriveWheelVoltages();
    @Log
    private DifferentialDriveWheelVoltages feedforwardVoltages = new DifferentialDriveWheelVoltages();

    @Log
    private final PIDController leftVelocityController = new PIDController(kP, kI, kD);
    @Log
    private final PIDController rightVelocityController = new PIDController(kP, kI, kD);

    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(kS, kV_LINEAR, kA_LINEAR);

    private final DifferentialDrivetrainSim drivetrainSim;

    private final SimDouble angleSim = new SimDouble(
            SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));

    private final SimDeviceSim leftMotorSim;
    private final SimDouble leftEncoderVelocitySim;
    private final SimDeviceSim rightMotorSim;
    private final SimDouble rightEncoderVelocitySim;

    // required for SysId, for whatever reason will not take values from a
    // SimDeviceSim
    private double leftEncoderVelocitySimValue = 0.0;
    private double rightEncoderVelocitySimValue = 0.0;

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
                                            (RobotBase.isReal() ? 12 : 1) * leftPrimaryMotor.getAppliedOutput(),
                                            Volts))
                                    .linearPosition(sysidPositionMeasure
                                            .mut_replace(leftPrimaryMotor.getEncoder().getPosition(), Meters))
                                    .linearVelocity(sysidVelocityMeasure
                                            .mut_replace(
                                                    RobotBase.isReal() ? leftPrimaryMotor.getEncoder().getVelocity()
                                                            : leftEncoderVelocitySimValue,
                                                    MetersPerSecond));
                            log.motor("right")
                                    .voltage(sysidAppliedVoltageMeasure.mut_replace(
                                            (RobotBase.isReal() ? 12 : 1) * rightPrimaryMotor.getAppliedOutput(),
                                            Volts))
                                    .linearPosition(sysidPositionMeasure
                                            .mut_replace(rightPrimaryMotor.getEncoder().getPosition(), Meters))
                                    .linearVelocity(sysidVelocityMeasure
                                            .mut_replace(
                                                    RobotBase.isReal() ? rightPrimaryMotor.getEncoder().getVelocity()
                                                            : rightEncoderVelocitySimValue,
                                                    MetersPerSecond));
                        },
                        this));

        leftMotorSim = new SimDeviceSim("SPARK MAX [" + LEFT_PRIMARY_MOTOR_ID + "]");
        leftEncoderVelocitySim = leftMotorSim.getDouble("Velocity");
        rightMotorSim = new SimDeviceSim("SPARK MAX [" + RIGHT_PRIMARY_MOTOR_ID + "]");
        rightEncoderVelocitySim = rightMotorSim.getDouble("Velocity");
        drivetrainSim = new DifferentialDrivetrainSim(GEARBOX_REPR, GEARING, MOI, MASS_KG, WHEEL_RADIUS,
                TRACK_WIDTH_METERS, VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

        AutoManager.getInstance().setResetOdometryConsumer(this::resetPoseEstimator);
    }

    @Override
    public void periodic() {
        updatePoseEstimator();
        drawRobotOnField(AutoManager.getInstance().getField());
    }

    private void drawRobotOnField(Field2d field) {
        field.setRobotPose(getPose());
    }

    @Override
    public void simulationPeriodic() {
        drivetrainSim.setInputs(
                leftPrimaryMotor.getAppliedOutput(),
                rightPrimaryMotor.getAppliedOutput());
        drivetrainSim.update(LOOP_TIME);

        leftPrimaryMotor.getEncoder().setPosition(drivetrainSim.getLeftPositionMeters());
        leftEncoderVelocitySim.set(drivetrainSim.getLeftVelocityMetersPerSecond());
        leftEncoderVelocitySimValue = drivetrainSim.getLeftVelocityMetersPerSecond();
        rightPrimaryMotor.getEncoder().setPosition(drivetrainSim.getRightPositionMeters());
        rightEncoderVelocitySim.set(drivetrainSim.getRightVelocityMetersPerSecond());
        rightEncoderVelocitySimValue = drivetrainSim.getRightVelocityMetersPerSecond();

        angleSim.set(-drivetrainSim.getHeading().getDegrees());
    }

    @Override
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public Rotation2d getRotation() {
        return gyro.getRotation2d();
    }

    @Override
    @Log
    public DifferentialDriveWheelPositions getWheelPositions() {
        return new DifferentialDriveWheelPositions(
                leftPrimaryMotor.getEncoder().getPosition(),
                rightPrimaryMotor.getEncoder().getPosition());
    }

    @Override
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
                leftPrimaryMotor.getEncoder().getVelocity(),
                rightPrimaryMotor.getEncoder().getVelocity());
    }

    @Override
    public DifferentialDriveWheelVoltages getWheelVoltages() {
        return new DifferentialDriveWheelVoltages(
                leftPrimaryMotor.getAppliedOutput(),
                rightPrimaryMotor.getAppliedOutput());
    }

    @Override
    public ChassisSpeeds getChassisSpeeds() {
        return KINEMATICS.toChassisSpeeds(getWheelSpeeds());
    }

    @Override
    public DifferentialDrivePoseEstimator getPoseEstimator() {
        return poseEstimator;
    }

    @Override
    public void updatePoseEstimator() {
        poseEstimator.update(getRotation(), getWheelPositions());
    }

    @Override
    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(getRotation(), getWheelPositions(), pose);
    }

    @Override
    public void resetGyro() {
        gyro.reset();
    }

    @Override
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

    @Override
    public void setCurrentLimit(int currentLimit) {
        leftPrimaryMotor.setSmartCurrentLimit(currentLimit);
        leftSecondaryMotor.setSmartCurrentLimit(currentLimit);
        rightPrimaryMotor.setSmartCurrentLimit(currentLimit);
        rightSecondaryMotor.setSmartCurrentLimit(currentLimit);
    }

    @Override
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

    @Override
    public void drive(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = KINEMATICS.toWheelSpeeds(speeds);
        wheelSpeeds.desaturate(MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        leftPrimaryMotor.setVoltage((wheelSpeeds.leftMetersPerSecond / MAX_DRIVING_VELOCITY_METERS_PER_SECOND) * 12.0);
        rightPrimaryMotor
                .setVoltage((wheelSpeeds.rightMetersPerSecond / MAX_DRIVING_VELOCITY_METERS_PER_SECOND) * 12.0);
    }

    @Override
    public void driveClosedLoop(ChassisSpeeds speeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = KINEMATICS.toWheelSpeeds(speeds);
        DifferentialDriveWheelSpeeds currentWheelSpeeds = getWheelSpeeds();
        wheelSpeeds.desaturate(MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        feedbackVoltages = new DifferentialDriveWheelVoltages(
                leftVelocityController.calculate(currentWheelSpeeds.leftMetersPerSecond,
                        wheelSpeeds.leftMetersPerSecond),
                rightVelocityController.calculate(currentWheelSpeeds.rightMetersPerSecond,
                        wheelSpeeds.rightMetersPerSecond));

        feedforwardVoltages = new DifferentialDriveWheelVoltages(
                feedforward.calculate(wheelSpeeds.leftMetersPerSecond),
                feedforward.calculate(wheelSpeeds.rightMetersPerSecond));

        leftPrimaryMotor.setVoltage(feedbackVoltages.left + feedforwardVoltages.left);
        rightPrimaryMotor.setVoltage(feedbackVoltages.right + feedforwardVoltages.right);
    }

    @Override
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

    @Override
    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathRamsete(
                path,
                this::getPose, // Robot pose supplier
                this::getChassisSpeeds, // Current ChassisSpeeds supplier
                this::driveClosedLoop, // Method that will drive the robot given ChassisSpeeds
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

    @Override
    public Command resetGyroCommand() {
        return Commands.runOnce(() -> this.resetGyro()).withName("drivetrain.resetGyro");
    }

    @Override
    public Command setCurrentLimitCommand(int currentLimit) {
        return Commands.runOnce(() -> this.setCurrentLimit(currentLimit)).withName("drivetrain.setCurrentLimit");
    }

    @Override
    public Command coastMotorsCommand() {
        return runEnd(
                () -> this.setMotorHoldModes(MotorHoldMode.COAST),
                () -> this.setMotorHoldModes(MotorHoldMode.BRAKE))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)
                .withName("drivetrain.coastMotors");
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
