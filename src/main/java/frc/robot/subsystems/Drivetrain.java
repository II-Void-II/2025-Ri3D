package frc.robot.subsystems;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.techhounds.houndutil.houndauto.AutoManager;
import com.techhounds.houndutil.houndlib.CurvatureDriveHelper;
import com.techhounds.houndutil.houndlib.MotorHoldMode;
import com.techhounds.houndutil.houndlib.subsystems.BaseDifferentialDrive;
import com.techhounds.houndutil.houndlog.annotations.Log;
import com.techhounds.houndutil.houndlog.annotations.LoggedObject;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelPositions;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import static frc.robot.Constants.Drivetrain.*;
import static frc.robot.Constants.LOOP_TIME;
import static frc.robot.Constants.Controls.*;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

@LoggedObject
public class Drivetrain extends SubsystemBase implements BaseDifferentialDrive {
    @Log
    private final SparkMax leftPrimaryMotor;
    @Log
    private final SparkMax leftSecondaryMotor;
    @Log
    private final SparkMax rightPrimaryMotor;
    @Log
    private final SparkMax rightSecondaryMotor;

    @Log
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    private final DifferentialDrivePoseEstimator poseEstimator;

    private final MutVoltage sysidAppliedVoltageMeasure = Volts.mutable(0);
    private final MutDistance sysidPositionMeasure = Meters.mutable(0);
    private final MutLinearVelocity sysidVelocityMeasure = MetersPerSecond.mutable(0);

    private final SysIdRoutine sysIdRoutine;
    private final SysIdRoutine sysIdRotationRoutine;

    @Log
    private DifferentialDriveWheelVoltages feedbackVoltages = new DifferentialDriveWheelVoltages();
    @Log
    private DifferentialDriveWheelVoltages feedforwardVoltages = new DifferentialDriveWheelVoltages();

    @Log
    private final PIDController leftVelocityController = new PIDController(VELOCITY_kP, VELOCITY_kI, VELOCITY_kD);
    @Log
    private final PIDController rightVelocityController = new PIDController(VELOCITY_kP, VELOCITY_kI, VELOCITY_kD);
    @Log
    private final ProfiledPIDController distanceController = new ProfiledPIDController(
            POSITION_kP, POSITION_kI, POSITION_kD, MOVEMENT_CONSTRAINTS);
    @Log
    private final ProfiledPIDController rotationController = new ProfiledPIDController(
            ROTATION_kP, ROTATION_kI, ROTATION_kD, ROTATION_CONSTRAINTS);

    private final DifferentialDriveFeedforward feedforward = new DifferentialDriveFeedforward(kV_LINEAR, kA_LINEAR,
            kV_ANGULAR, kA_ANGULAR, TRACK_WIDTH_METERS);

    private DifferentialDriveWheelSpeeds lastWheelSpeeds = new DifferentialDriveWheelSpeeds();
    @Log
    private DifferentialDriveWheelSpeeds commandedWheelSpeeds = new DifferentialDriveWheelSpeeds();

    private final DifferentialDrivetrainSim drivetrainSim;

    private final SimDouble angleSim = new SimDouble(
            SimDeviceDataJNI.getSimValueHandle(SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]"), "Yaw"));

    private final SimDeviceSim leftMotorSim;
    private final SimDouble leftEncoderVelocitySim;
    private final SimDeviceSim rightMotorSim;
    private final SimDouble rightEncoderVelocitySim;

    private SparkMaxConfig motorConfig;

    // required for SysId, for whatever reason will not take values from a
    // SimDeviceSim
    private double leftEncoderVelocitySimValue = 0.0;
    private double rightEncoderVelocitySimValue = 0.0;
    private CurvatureDriveHelper curvatureDriveHelper = new CurvatureDriveHelper();

    public Drivetrain() {

        motorConfig = new SparkMaxConfig();

        motorConfig
                .inverted(LEFT_DRIVE_MOTORS_INVERTED)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(CURRENT_LIMIT);
        motorConfig.encoder
                .positionConversionFactor(ENCODER_ROTATIONS_TO_METERS)
                .velocityConversionFactor(ENCODER_ROTATIONS_TO_METERS / 60.0);

        leftPrimaryMotor = new SparkMax(LEFT_PRIMARY_MOTOR_ID, MotorType.kBrushless);
        leftPrimaryMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorConfig.follow(leftPrimaryMotor);
        leftSecondaryMotor = new SparkMax(LEFT_SECONDARY_MOTOR_ID, MotorType.kBrushless);
        leftSecondaryMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorConfig.disableFollowerMode();
        motorConfig.inverted(RIGHT_DRIVE_MOTORS_INVERTED);
        rightPrimaryMotor = new SparkMax(RIGHT_PRIMARY_MOTOR_ID, MotorType.kBrushless);
        rightPrimaryMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        motorConfig.follow(rightPrimaryMotor);
        rightSecondaryMotor = new SparkMax(RIGHT_SECONDARY_MOTOR_ID, MotorType.kBrushless);
        rightSecondaryMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        poseEstimator = new DifferentialDrivePoseEstimator(
                KINEMATICS, gyro.getRotation2d(), getWheelPositions().leftMeters, getWheelPositions().rightMeters,
                new Pose2d());

        sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(1).per(Seconds), Volts.of(8), null, null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> setVoltage(volts.magnitude()),
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

        sysIdRotationRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(Volts.of(1).per(Seconds), Volts.of(8), null, null),
                new SysIdRoutine.Mechanism(
                        (Voltage volts) -> {
                            leftPrimaryMotor.setVoltage(volts.magnitude());
                            rightPrimaryMotor.setVoltage(-volts.magnitude());
                        },
                        log -> {
                            log.motor("left")
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
        // drivetrainSim = new DifferentialDrivetrainSim(GEARBOX_REPR, GEARING, MOI,
        // MASS_KG, WHEEL_RADIUS_METERS,
        // TRACK_WIDTH_METERS, VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
        drivetrainSim = new DifferentialDrivetrainSim(GEARBOX_REPR, GEARING, MOI, MASS_KG, WHEEL_RADIUS_METERS,
                TRACK_WIDTH_METERS, null);

        AutoManager.getInstance().setResetOdometryConsumer(this::resetPoseEstimator);

        distanceController.setTolerance(0.05);
        rotationController.setTolerance(0.02);
        rotationController.enableContinuousInput(0, 2 * Math.PI);
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

    @Log
    @Override
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public Rotation2d getRotation() {
        // not directly from gyro because pose estimator resets will introduce an offset
        return poseEstimator.getEstimatedPosition().getRotation();
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
        poseEstimator.update(gyro.getRotation2d(), getWheelPositions());
    }

    @Override
    public void resetPoseEstimator(Pose2d pose) {
        poseEstimator.resetPosition(gyro.getRotation2d(), getWheelPositions(), pose);
    }

    @Override
    public void resetGyro() {
        gyro.reset();
    }

    @Override
    public void setMotorHoldModes(MotorHoldMode motorHoldMode) {
        if (motorHoldMode == MotorHoldMode.COAST) {
            SparkMaxConfig coastConfig = new SparkMaxConfig();
            coastConfig.idleMode(IdleMode.kCoast);

            leftPrimaryMotor.configure(coastConfig, ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
            leftSecondaryMotor.configure(coastConfig, ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
            rightPrimaryMotor.configure(coastConfig, ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
            rightSecondaryMotor.configure(coastConfig, ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
        } else {
            SparkMaxConfig brakeConfig = new SparkMaxConfig();
            brakeConfig.idleMode(IdleMode.kBrake);

            leftPrimaryMotor.configure(brakeConfig, ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
            leftSecondaryMotor.configure(brakeConfig, ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
            rightPrimaryMotor.configure(brakeConfig, ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
            rightSecondaryMotor.configure(brakeConfig, ResetMode.kNoResetSafeParameters,
                    PersistMode.kNoPersistParameters);
        }
    }

    @Override
    public void setCurrentLimit(int currentLimit) {
        SparkMaxConfig currentConfig = new SparkMaxConfig();
        currentConfig.smartCurrentLimit(currentLimit);

        leftPrimaryMotor.configure(currentConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        leftSecondaryMotor.configure(currentConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        rightPrimaryMotor.configure(currentConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
        rightSecondaryMotor.configure(currentConfig, ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void stop() {
        leftPrimaryMotor.setVoltage(0);
        leftSecondaryMotor.setVoltage(0);
        rightPrimaryMotor.setVoltage(0);
        rightSecondaryMotor.setVoltage(0);
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
        commandedWheelSpeeds = KINEMATICS.toWheelSpeeds(speeds);
        commandedWheelSpeeds.desaturate(MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        feedbackVoltages = new DifferentialDriveWheelVoltages(
                leftVelocityController.calculate(lastWheelSpeeds.leftMetersPerSecond,
                        commandedWheelSpeeds.leftMetersPerSecond),
                rightVelocityController.calculate(lastWheelSpeeds.rightMetersPerSecond,
                        commandedWheelSpeeds.rightMetersPerSecond));

        feedforwardVoltages = feedforward.calculate(lastWheelSpeeds.leftMetersPerSecond,
                commandedWheelSpeeds.leftMetersPerSecond,
                lastWheelSpeeds.rightMetersPerSecond,
                commandedWheelSpeeds.rightMetersPerSecond, LOOP_TIME);

        leftPrimaryMotor.setVoltage(MathUtil.clamp(feedbackVoltages.left + feedforwardVoltages.left, -12, 12));
        rightPrimaryMotor.setVoltage(MathUtil.clamp(feedbackVoltages.right + feedforwardVoltages.right, -12, 12));
        lastWheelSpeeds = commandedWheelSpeeds;
    }

    public void driveClosedLoop(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {

        commandedWheelSpeeds = KINEMATICS.toWheelSpeeds(speeds);
        commandedWheelSpeeds.desaturate(MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

        feedbackVoltages = new DifferentialDriveWheelVoltages(
                leftVelocityController.calculate(lastWheelSpeeds.leftMetersPerSecond,
                        commandedWheelSpeeds.leftMetersPerSecond),
                rightVelocityController.calculate(lastWheelSpeeds.rightMetersPerSecond,
                        commandedWheelSpeeds.rightMetersPerSecond));

        LinearAcceleration[] feedforwardAccelerations = feedforwards.accelerations();

        DifferentialDriveWheelVoltages feedforwardVoltages = new DifferentialDriveWheelVoltages(
                commandedWheelSpeeds.leftMetersPerSecond * feedforward.m_kVLinear
                        + feedforwardAccelerations[0].magnitude() * feedforward.m_kALinear,
                commandedWheelSpeeds.rightMetersPerSecond * feedforward.m_kVLinear
                        + feedforwardAccelerations[1].magnitude() * feedforward.m_kALinear);

        leftPrimaryMotor.setVoltage(MathUtil.clamp(feedbackVoltages.left + feedforwardVoltages.left, -12, 12));
        rightPrimaryMotor.setVoltage(MathUtil.clamp(feedbackVoltages.right + feedforwardVoltages.right, -12, 12));
        lastWheelSpeeds = commandedWheelSpeeds;
    }

    public Command teleopDriveCommand(DoubleSupplier leftStickThrustSupplier, DoubleSupplier rightStickThrustSupplier,
            DoubleSupplier rightStickRotationSupplier, BooleanSupplier quickTurnSupplier,
            Supplier<DifferentialDriveMode> driveModeSupplier) {
        SlewRateLimiter leftThrustLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter rightThrustLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);
        SlewRateLimiter rotationLimiter = new SlewRateLimiter(JOYSTICK_INPUT_RATE_LIMIT);

        return run(() -> {
            double leftThrust = leftStickThrustSupplier.getAsDouble();
            double rightThrust = rightStickThrustSupplier.getAsDouble();
            double rightRotation = rightStickRotationSupplier.getAsDouble();
            rightRotation *= JOYSTICK_ROT_LIMIT;
            DifferentialDriveMode driveMode = driveModeSupplier.get();

            leftThrust = Math.copySign(Math.pow(leftThrust, JOYSTICK_CURVE_EXP), leftThrust);
            rightThrust = Math.copySign(Math.pow(rightThrust, JOYSTICK_CURVE_EXP), rightThrust);
            rightRotation = Math.copySign(Math.pow(rightRotation, JOYSTICK_ROT_CURVE_EXP), rightRotation);

            leftThrust = leftThrustLimiter.calculate(leftThrust);
            rightThrust = rightThrustLimiter.calculate(rightThrust);
            rightRotation = rotationLimiter.calculate(rightRotation);

            switch (driveMode) {
                case ARCADE -> {
                    leftThrust *= MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
                    rightThrust *= MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
                    rightRotation *= MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

                    drive(new ChassisSpeeds(leftThrust, 0, rightRotation));
                }
                case TANK -> {
                    leftThrust *= MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
                    rightThrust *= MAX_DRIVING_VELOCITY_METERS_PER_SECOND;
                    rightRotation *= MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

                    // the speeds are initially values from -1.0 to 1.0, so we multiply by the max
                    // physical velocity to output in m/s.
                    ChassisSpeeds speeds = KINEMATICS.toChassisSpeeds(new DifferentialDriveWheelSpeeds(
                            leftThrust, rightThrust));

                    drive(speeds);
                }
                case CURVATURE -> {
                    boolean quickTurn = quickTurnSupplier.getAsBoolean();
                    DifferentialDriveWheelSpeeds wheelSpeeds = curvatureDriveHelper.cheesyDrive(leftThrust,
                            -rightRotation, quickTurn, MAX_DRIVING_VELOCITY_METERS_PER_SECOND);

                    ChassisSpeeds speeds = KINEMATICS.toChassisSpeeds(wheelSpeeds);

                    drive(speeds);
                }
            }
        }).withName("drivetrain.teleopDrive");
    }

    @Override
    public Command teleopDriveCommand(DoubleSupplier leftStickThrustSupplier, DoubleSupplier rightStickThrustSupplier,
            DoubleSupplier rightStickRotationSupplier,
            Supplier<DifferentialDriveMode> driveModeSupplier) {
        return teleopDriveCommand(leftStickThrustSupplier, rightStickThrustSupplier, rightStickRotationSupplier, null,
                driveModeSupplier);
    }

    @Override
    public Command followPathCommand(PathPlannerPath path) {
        return new FollowPathCommand(
                path,
                this::getPose,
                this::getChassisSpeeds,
                this::driveClosedLoop,
                new PPLTVController(0.02),
                ROBOT_CONFIG, // The robot configuration
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, this)
                .andThen(this::stop, this)
                .withName("drivetrain.followPath");
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

    public Command driveDistanceCommand(DoubleSupplier distance) {
        return runOnce(() -> {
            distanceController.reset(leftPrimaryMotor.getEncoder().getPosition());
            distanceController.setGoal(leftPrimaryMotor.getEncoder().getPosition() + distance.getAsDouble());
            // easiest way to keep chassis straight
            rotationController.reset(getRotation().getRadians());
            rotationController.setGoal(getRotation().getRadians());
        }).andThen(runEnd(() -> {
            // if gains are in terms of velocity, can just add setpoint velocity to the
            // calculated velocity to overcome current error
            double velocity = distanceController.getSetpoint().velocity
                    + distanceController.calculate(leftPrimaryMotor.getEncoder().getPosition());
            ChassisSpeeds speeds = new ChassisSpeeds(velocity,
                    0, rotationController.calculate(getRotation().getRadians()));
            driveClosedLoop(speeds);
        }, this::stop))
                .until(distanceController::atGoal)
                .withName("drivetrain.driveDistance");
    }

    public Command rotateToAngle(Supplier<Rotation2d> angle) {
        return runOnce(() -> {
            rotationController.reset(getRotation().getRadians());
            rotationController.setGoal(angle.get().getRadians());
        }).andThen(runEnd(() -> {
            // if gains are in terms of velocity, can just add setpoint velocity to the
            // calculated velocity to overcome current error
            double rotationalVelocity = rotationController.getSetpoint().velocity
                    + rotationController.calculate(getRotation().getRadians());
            ChassisSpeeds speeds = new ChassisSpeeds(0, 0, rotationalVelocity);
            driveClosedLoop(speeds);
        }, this::stop))
                .until(rotationController::atGoal)
                .withName("drivetrain.rotateToAngle");
    }

    public Command moveVoltageTimeCommand(double voltage, double time) {
        return run(() -> setVoltage(voltage)).andThen(this::stop).withTimeout(time);
    }

    public Command sysIdQuasistaticCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.quasistatic(direction).withName("drivetrain.sysIdQuasistatic");
    }

    public Command sysIdDynamicCommand(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction).withName("drivetrain.sysIdDynamic");
    }

    public Command sysIdQuasistaticRotationCommand(SysIdRoutine.Direction direction) {
        return sysIdRotationRoutine.quasistatic(direction).withName("drivetrain.sysIdQuasistaticRotation");
    }

    public Command sysIdDynamicRotationCommand(SysIdRoutine.Direction direction) {
        return sysIdRotationRoutine.dynamic(direction).withName("drivetrain.sysIdDynamicRotation");
    }

}
