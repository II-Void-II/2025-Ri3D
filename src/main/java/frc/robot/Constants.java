package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

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

        public static final double GEARING = 8.45;
        public static final double WHEEL_RADIUS_METERS = Units.inchesToMeters(3); // TODO
        public static final double WHEEL_CIRCUMFERENCE = 2 * Math.PI * WHEEL_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = WHEEL_CIRCUMFERENCE / GEARING;

        public static final int CURRENT_LIMIT = 60; // TODO
        public static final DCMotor GEARBOX_REPR = DCMotor.getNEO(2);
        public static final double MOI = 2;
        public static final double MASS_KG = 20; // TODO

        public static final double VELOCITY_kP = RobotBase.isReal() ? 1.5 : 1.4078;
        public static final double VELOCITY_kI = 0.0;
        public static final double VELOCITY_kD = 0.0;
        public static final double kS = RobotBase.isReal() ? 0.29971 : 0.057644;
        public static final double kV_LINEAR = RobotBase.isReal() ? 2.4 : 2.159;
        public static final double kA_LINEAR = RobotBase.isReal() ? 0.56555 : 0.24174;
        public static final double kV_ANGULAR = RobotBase.isReal() ? 1.2427 : 2.1403;
        public static final double kA_ANGULAR = RobotBase.isReal() ? 0.0859 : 0.42056;

        public static final double POSITION_kP = 1.5;
        public static final double POSITION_kI = 0.0;
        public static final double POSITION_kD = 0.0;
        public static final double ROTATION_kP = 2.0;
        public static final double ROTATION_kI = 0.0;
        public static final double ROTATION_kD = 0.0;

        public static final double RAMSETE_B = 4; // TODO
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
            BOTTOM(0),
            INTAKE(0.437),
            L2(0.3),
            L3(0.8),
            L4(1.57),
            TOP(1.57);

            public final double value;

            private ElevatorPosition(double value) {
                this.value = value;
            }
        }

        public static final double SCORING_MOVEMENT = -0.25;

        public static final int MOTOR_ID = 5;
        public static final boolean MOTOR_INVERTED = false;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
        public static final double GEARING = 3.0;
        public static final double MASS_KG = Units.lbsToKilograms(20);
        public static final double DRUM_RADIUS_METERS = Units.inchesToMeters(1.05) / 2.0; // TODO
        public static final double DRUM_CIRCUMFERENCE = 2.0 * Math.PI * DRUM_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = DRUM_CIRCUMFERENCE * GEARING;

        public static final double MIN_HEIGHT_METERS = 0.005; // TODO
        public static final double MAX_HEIGHT_METERS = 1.57; // TODO

        public static final int CURRENT_LIMIT = 60;

        public static final double kP = 5; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
        public static final double kS = 0.013602; // TODO
        public static final double kG = 1.8233; // TODO
        public static final double kV = 4.4607; // TODO
        public static final double kA = 0.18111; // TODO
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 3; // TODO
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 6; // TODO
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public static final class Arm {
        public static enum ArmPosition {
            BOTTOM(-Math.PI / 2.0),
            HORIZONTAL(0),
            SCORING(Units.degreesToRadians(30)),
            TOP(Math.PI / 2.0);

            public final double value;

            private ArmPosition(double value) {
                this.value = value;
            }
        }

        public static final double SCORING_MOVEMENT = -0.8;

        public static final int MOTOR_ID = 6;
        public static final boolean MOTOR_INVERTED = false;

        public static final DCMotor MOTOR_GEARBOX_REPR = DCMotor.getNEO(1);
        public static final double GEARING = 40.0; // TODO
        public static final double MASS_KG = Units.lbsToKilograms(10); // TODO
        public static final double COM_DISTANCE_METERS = Units.inchesToMeters(6); // TODO
        public static final double MOI = SingleJointedArmSim.estimateMOI(COM_DISTANCE_METERS, MASS_KG);
        public static final double ENCODER_ROTATIONS_TO_METERS = 2 * Math.PI / GEARING;

        public static final double MIN_ANGLE_RADIANS = -Math.PI / 2.0;
        public static final double MAX_ANGLE_RADIANS = Math.PI / 2.0;

        public static final int CURRENT_LIMIT = 50;

        public static final double kP = 2.0; // TODO
        public static final double kI = 0; // TODO
        public static final double kD = 0; // TODO
        public static final double kS = 0.017964; // TODO
        public static final double kG = 0.38922; // TODO
        public static final double kV = 0.78395;// TODO
        public static final double kA = 0.015936;// TODO
        public static final double TOLERANCE = 0.02;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 30; // TODO
        public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 60; // TODO
        public static final TrapezoidProfile.Constraints MOVEMENT_CONSTRAINTS = new TrapezoidProfile.Constraints(
                MAX_VELOCITY_METERS_PER_SECOND, MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);
    }

    public static final class Intake {
        public static final int MOTOR_ID = 7;
        public static final boolean MOTOR_INVERTED = false;
        public static final int CURRENT_LIMIT = 20;
    }

    public static final class Climber {
        public static final int MOTOR_ID = 8;
        public static final boolean MOTOR_INVERTED = false;
        public static final int CURRENT_LIMIT = 60;

        public static final double MIN_POSITION_METERS = 0.0;
        public static final double MAX_POSITION_METERS = 1.0; // TODO

        public static final double GEARING = 64.0;
        public static final double MASS_KG = Units.lbsToKilograms(80); // robot weight
        public static final double SPOOL_RADIUS_METERS = Units.inchesToMeters(0.5);
        public static final double SPOOL_CIRCUMFERENCE = 2.0 * Math.PI * SPOOL_RADIUS_METERS;
        public static final double ENCODER_ROTATIONS_TO_METERS = SPOOL_CIRCUMFERENCE * GEARING;

    }

    public static final class Controls {
        public static final double JOYSTICK_INPUT_RATE_LIMIT = 15.0;
        public static final double JOYSTICK_INPUT_DEADBAND = 0.05;
        public static final double JOYSTICK_CURVE_EXP = 2;
        public static final double JOYSTICK_ROT_CURVE_EXP = 2;
    }
}
