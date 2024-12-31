package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final double LOOP_TIME = 0.020; // 20ms

    public static final class Drivetrain {

        public static final int LEFT_PRIMARY_MOTOR_ID = 1;
        public static final int LEFT_SECONDARY_MOTOR_ID = 2;
        public static final int RIGHT_PRIMARY_MOTOR_ID = 3;
        public static final int RIGHT_SECONDARY_MOTOR_ID = 4;

        public static final boolean LEFT_DRIVE_MOTORS_INVERTED = false; // TODO
        public static final boolean RIGHT_DRIVE_MOTORS_INVERTED = true; // TODO

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH_METERS = 0.527; // TODO
        /** Distance between front and back wheels. */
        public static final double WHEEL_BASE_METERS = 0.527; // TODO
        public static final double DRIVE_BASE_RADIUS_METERS = 0.3727; // TODO

        public static final double GEARING = 8.45;
        public static final double WHEEL_CIRCUMFERENCE = Math.PI * Units.inchesToMeters(6); // TODO
        public static final double ENCODER_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / GEARING;

        public static final int CURRENT_LIMIT = 60; // TODO
        public static final DCMotor GEARBOX_REPR = DCMotor.getNEO(2);
        public static final double MOI = 0.01;

        public static final double kP = 0.396252;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0519492;
        public static final double kV_LINEAR = 1.135188;
        public static final double kA_LINEAR = 0.252588;
        public static final double kV_ANGULAR = 0;
        public static final double kA_ANGULAR = 0;

        public static final double RAMSETE_B = 2; // TODO
        public static final double RAMSETE_ZETA = 0.7; // TODO

        public static final double MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 5; // TODO
        public static final double MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 5; // TODO
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 10;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 30;

        public static final double PATH_FOLLOWING_TRANSLATION_kP = 8.0; // TODO
        public static final double PATH_FOLLOWING_ROTATION_kP = 8.0; // TODO

        public static final double XY_kP = 1.4; // TODO
        public static final double XY_kI = 0;
        public static final double XY_kD = 0.05; // TODO
        public static final TrapezoidProfile.Constraints XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED);

        public static final double THETA_kP = 1.3; // TODO
        public static final double THETA_kI = 0;
        public static final double THETA_kD = 0.05; // TODO
        public static final TrapezoidProfile.Constraints THETA_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);
    }

    public static final class Controls {
        public static final double JOYSTICK_INPUT_RATE_LIMIT = 15.0;
        public static final double JOYSTICK_INPUT_DEADBAND = 0.05;
        public static final double JOYSTICK_CURVE_EXP = 2;
        public static final double JOYSTICK_ROT_CURVE_EXP = 2;
    }
}
