// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
//import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
//import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
//import edu.wpi.first.math.numbers.N1;
//import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
//import edu.wpi.first.math.Matrix;
import swervelib.math.Matter;

public final class Constants {
  public static final class LoggedDashboard{
    public static final boolean tuningMode = true; 
  }

  public static class Swerve {
    public static final double ROBOT_MASS = 110 * 0.453592; // 32lbs * kg per pound
    public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
    public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
    public static final double MAX_SPEED  = Units.feetToMeters(14.5);
    public static final double WHEEL_LOCK_TIME = 10; // seconds
    public static final double angularVelocityCoeff = 0.1;

    public static class Auton {
      public static class AngleControl {
          public static final double kP = 0.03;
          public static final double kI = 0.0;
          public static final double kD = 0.01;
      }
      public static class TranslationControl {
          public static final double kP = 0.001;
          public static final double kI = 0.0;
          public static final double kD = 0.1;
      }
    }
  }
  
  public static class Vision {
        public static final AprilTagFieldLayout kTagLayout = AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        
        // public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        // public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  }

  public static class Controller {
    public static class Driver {
      public static final double LEFT_X_DEADBAND = 0.1;
      public static final double LEFT_Y_DEADBAND = 0.1;
      public static final double RIGHT_X_DEADBAND = 0.1;
      public static final double RIGHT_Y_DEADBAND = 0.1;
      public static final double TURN_CONSTANT = 6;
    }
    public static class Operator {
      public static final double TRIGGER_DEADBAND = 0.06;
      public static final double JOYSTICK_DEADBAND = 0.06;
    }
  }

  public static class Mapping {
    public static class Controllers {
      public static final int driver = 0;
      public static final int operator = 1;
    }

    public static class Pigeon2 {
      public static final int gyro = 0;
    }
  }

  public static class Generic {
    public static int timeoutMs = 1000;
  }

  public static class GamePieces {
    public static class Speaker {
      public static final double speakerAprilTagHeightCM = 145.0975;
      public static final Pose3d kSpeakerCenterBlue = new Pose3d(0.2167, 5.549, 2.12, new Rotation3d());
      public static final Pose3d kSpeakerCenterRed = new Pose3d(16.3, 5.549, 2.12, new Rotation3d());
      // Retune for STEMLEY
      public static final double speakerOpeningHeightCM = 205;
      public static final double angle = 60.0;
    }

    public static class Amp {
      public static final double angleToshoot = 64;
      public static final double speedToShoot = 14;
    }
  }
}
