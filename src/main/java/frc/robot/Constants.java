// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static enum CAN_IDs {


    //SWERVE DRIVE
    FRONT_LEFT_TURN(14),
    FRONT_LEFT_DRIVE(15),
    FRONT_RIGHT_TURN(10),
    FRONT_RIGHT_DRIVE(11),
    BACK_RIGHT_TURN(8),
    BACK_RIGHT_DRIVE(9),
    BACK_LEFT_TURN(12),
    BACK_LEFT_DRIVE(13),
    INTAKE(2),
    CONVEYOR(4),
    FEEDER(5),
    FLYWHEEL(6),
    FLYWHEEL_FOLLOWER(7),
    PIGEON(0);


    public int ID; 
    private CAN_IDs(int ID){
      this.ID=ID;
    }
  }

  public static class SwerveDriveConstants {

    public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI;

    private static final double ANGULAR_OFFSET_COMPENSATION = Math.PI / 4;
    public static final double FRONT_LEFT_ANGULAR_OFFSET = (-Math.PI / 2) + ANGULAR_OFFSET_COMPENSATION * 2.5;
    public static final double FRONT_RIGHT_ANGULAR_OFFSET = 3 * Math.PI / 2 + ANGULAR_OFFSET_COMPENSATION / 2 ;
    public static final double BACK_LEFT_ANGULAR_OFFSET = ANGULAR_OFFSET_COMPENSATION - Math.PI / 2;
    public static final double BACK_RIGHT_ANGULAR_OFFSET = -(Math.PI / 2) + ANGULAR_OFFSET_COMPENSATION;

    public static final boolean FRONT_LEFT_DRIVING_ON_BOTTOM = true;
    public static final boolean BACK_LEFT_DRIVING_ON_BOTTOM = true;
    public static final boolean FRONT_RIGHT_DRIVING_ON_BOTTOM = true;
    public static final boolean BACK_RIGHT_DRIVING_ON_BOTTOM = true;

    public static final boolean FRONT_LEFT_TURNING_ON_BOTTOM = true;
    public static final boolean BACK_LEFT_TURNING_ON_BOTTOM = true;
    public static final boolean FRONT_RIGHT_TURNING_ON_BOTTOM = true;
    public static final boolean BACK_RIGHT_TURNING_ON_BOTTOM = true;

    
    public static final double TRACK_WIDTH = Units.inchesToMeters(22.5);
    public static final double WHEEL_BASE = Units.inchesToMeters(22.5);
    
    public static final SwerveDriveKinematics DRIVE_KINEMATICS =
        new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    public static final boolean GYRO_REVERSED = false;

    public static enum DriveSpeeds {
      SLOW(           0.05, 
                  0.1),
      MEDIUM(         0.5,
                  0.2),
      FAST(           1,
                  1);

      public double linearMultiplier;
      public double rotationalMultiplier;

      private DriveSpeeds(double linearMultiplier, double rotationalMultiplier) {
        this.linearMultiplier     = linearMultiplier;
        this.rotationalMultiplier = rotationalMultiplier; 
      }
    }

    
  }

  public static class ModuleConstants {

    public static final int DRIVING_MOTOR_PINION_TEETH = 12;

      public static final double WHEEL_DIAMETER_METERS = Units.inchesToMeters(4);

      public static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_METERS * Math.PI;


      public static final double DRIVING_WHEEL_BEVEL_GEAR_TEETH = 45.0;
      public static final double DRIVING_WHEEL_FIRST_STAGE_SPUR_GEAR_TEETH = 30.0;
      public static final double DRIVING_MOTOR_BEVEL_PINION_TEETH = 15.0;
      public static final double DRIVING_MOTOR_REDUCTION = (DRIVING_WHEEL_BEVEL_GEAR_TEETH * DRIVING_WHEEL_FIRST_STAGE_SPUR_GEAR_TEETH)
          / (DRIVING_MOTOR_PINION_TEETH * DRIVING_MOTOR_BEVEL_PINION_TEETH);
    
         public static final double DRIVING_MOTOR_FREE_SPEED_RPS = NeoMotorConstants.NEO_FREE_SPEED_RPM / 60;
         public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE)
          / DRIVING_MOTOR_REDUCTION;
         
    
        }

    public static final class NeoMotorConstants {
      public static final double NEO_FREE_SPEED_RPM = 5676;
      public static final double VORTEX_KV = 565;
      
    }

    public static final class LauncherSubsystemConstants {

      public static final class FeederSetPoints {
        public static final double FEED         = 0.95;
        public static final double FEED_REVERSE = -0.5;
      }

      public static final class FlywheelSetPoints {
        public static final double LAUNCH_RPM = 5000;
        public static final double VELOCITY_TOLERANCE = 100;
      }
    }

    public static final class IntakeSubsystemConstants {
      public static final class IntakeSetpoints{
        public static final double INTAKE = 0.8;
        public static final double EXTAKE = -0.6;
      }

      public static final class ConveyorSetpoints{
        public static final double INTAKE = 0.7;
        public static final double EXTAKE = -0.7;
        public static final double EXTAKE_SLOW = -0.3;
      }


    }

    public static final class OIConstants {
      public static final int kDriverControllerPort = 0;
      public static final double kDriveDeadband = 0.1;
      public static final double kTriggerButtonThreshold = 0.2;
    }

    public static final class AutoConstants {
      public static final double MAX_SPEED_METERS_PER_SECOND = 3;
      public static final double MAX_ACCELRATION_MPS_SQUARED = 3;
      public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
      public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

      public static final double kPXController = 1;
      public static final double kPYController = 1;
      public static final double kPThetaController = 1;

      // Constraint for the motion profiled robot angle controller
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
    }

}

