package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.math.Matter;

public final class Constants {

    public static final double ROBOT_MASS = 48.0;
    public static final Matter CHASSIS = new Matter(
            new Translation3d(Units.inchesToMeters(27), Units.inchesToMeters(27), Units.inchesToMeters(40)),
            ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED = Units.feetToMeters(12.5);
    // Maximum speed of the robot in meters per second, used to limit acceleration.

    public static final class LoggedDashboard {
        public static final boolean TUNING_MODE = false;
    }

    public static final class Swerve {
        // Hold time on motor brakes when disabled
        public static final double WHEEL_LOCK_TIME = 10; // seconds

        public static final class Align {
            public static final class Heading {
                public static final double TOLERANCE = 1; // in degrees
            }

            public static final class Translation {
                public static final double TOLERANCE = Units.inchesToMeters(1.0);
            }
        }

        public static class Auton {
            public static class Angle {
                public static final double kP = 5.0;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
            }

            public static class Translation {
                public static final double kP = 2.5;
                public static final double kI = 0.0;
                public static final double kD = 0.0;
            }
        }
    }

    public static class Driver {
        // Joystick Deadband
        public static final double DEADBAND = 0.0;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double TURN_CONSTANT = 6;
    }

    public static class Pivot {
        // PID Values
        public static final double Pivot_kP = 0.05;
        public static final double Pivot_kI = 0.0;
        public static final double Pivot_kD = 0.0;

        // Mech properties
        public static final double Gear_Ratio = 1.0 / 1.0;
        public static final double Max_Angle = 90.0;
        public static final double Pivot_EncoderTicksPerRevolution = 4096.0;

        // FF Gains
        public static final double Pivot_kS = 0.2;
        public static final double Pivot_kG = 0.9;
        public static final double Pivot_kV = 1.5;
        public static final double Pivot_kA = 0.0;

        // Tp Constraints
        public static final double PivotMaxVelRadPerSec = Math.toRadians(180.0); // 180 deg/s
        public static final double PivotMaxAccelRadPerSec2 = Math.toRadians(360.0); // 360 deg/s^2

        public static final double ROTATION_TOLERANCE = 0.1;

        // Controller tolerances
        public static final double PivotPositionToleranceDeg = 1.0;
        public static final double PivotVelocityToleranceDegPerSec = 5.0;
    }

    public static class EndEffector {
        // Motor Speeds (0-1)
        public static final double IntakeSpeed = 0.5;
        public static final double ScoreSpeed = -0.25;

        // Motor PID Values
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
    }

}