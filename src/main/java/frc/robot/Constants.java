package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class Constants {
    public static final double LOOP_TIME = 0.020; // 20ms

    public static final class Drivetrain {

        public static final int LEFT_PRIMARY_MOTOR_ID = 3;
        public static final int LEFT_SECONDARY_MOTOR_ID = 4;
        public static final int RIGHT_PRIMARY_MOTOR_ID = 1;
        public static final int RIGHT_SECONDARY_MOTOR_ID = 2;

        public static final boolean LEFT_DRIVE_MOTORS_INVERTED = false; // TODO
        public static final boolean RIGHT_DRIVE_MOTORS_INVERTED = true; // TODO

        /** Distance between left and right wheels. */
        public static final double TRACK_WIDTH_METERS = Units.inchesToMeters(22); // TODO
        public static final double DRIVE_BASE_RADIUS_METERS = 0.3727; // TODO

        public static final double GEARING = 8.45;
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3); // TODO
        public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / GEARING;

        public static final int CURRENT_LIMIT = 60; // TODO
        public static final DCMotor GEARBOX_REPR = DCMotor.getNEO(2);
        public static final double MOI = 0.01;
        public static final double MASS_KG = 20; // TODO

        public static final double VELOCITY_kP = 1.5;
        public static final double VELOCITY_kI = 0.0;
        public static final double VELOCITY_kD = 0.0;
        public static final double kS = 0.29971;
        public static final double kV_LINEAR = 2.4;
        public static final double kA_LINEAR = 0.56555;
        public static final double kV_ANGULAR = 1.2427;
        public static final double kA_ANGULAR = 0.0859;

        public static final double POSITION_kP = 1.5;
        public static final double POSITION_kI = 0.0;
        public static final double POSITION_kD = 0.0;
        public static final double ROTATION_kP = 2.0;
        public static final double ROTATION_kI = 0.0;
        public static final double ROTATION_kD = 0.0;

        public static final double RAMSETE_B = 2; // TODO
        public static final double RAMSETE_ZETA = 0.7; // TODO

        public static final double MAX_DRIVING_VELOCITY_METERS_PER_SECOND = 5;
        public static final double MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED = 8; // TODO
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 20;
        public static final double MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 40;

        public static final double PATH_FOLLOWING_TRANSLATION_kP = 8.0; // TODO
        public static final double PATH_FOLLOWING_ROTATION_kP = 8.0; // TODO

        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_DRIVING_VELOCITY_METERS_PER_SECOND,
                MAX_DRIVING_ACCELERATION_METERS_PER_SECOND_SQUARED);
        public static final TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                MAX_ANGULAR_ACCELERATION_RADIANS_PER_SECOND_SQUARED);

        public static final DifferentialDriveKinematics KINEMATICS = new DifferentialDriveKinematics(
                TRACK_WIDTH_METERS);
    }

    public static final class Elevator {
        public static enum ElevatorPosition {
            BOTTOM(0);

            public final double value;

            private ElevatorPosition(double value) {
                this.value = value;
            }
        }
    }

    public static final class Arm {
        public static enum ArmPosition {
            BOTTOM(0);

            public final double value;

            private ArmPosition(double value) {
                this.value = value;
            }
        }
    }

    public static final class Controls {
        public static final double JOYSTICK_INPUT_RATE_LIMIT = 15.0;
        public static final double JOYSTICK_INPUT_DEADBAND = 0.05;
        public static final double JOYSTICK_CURVE_EXP = 2;
        public static final double JOYSTICK_ROT_CURVE_EXP = 2;
    }
}
