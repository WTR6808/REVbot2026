// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int kDriverControllerPort = 0;
  }

  public static enum CAN_IDs {


    //SWERVE DRIVE
    FRONT_LEFT_TURN(40),
    FRONT_LEFT_DRIVE(41),
    FRONT_RIGHT_TURN(42),
    FRONT_RIGHT_DRIVE(43),
    BACK_RIGHT_TURN(44),
    BACK_RIGHT_DRIVE(45),
    BACK_LEFT_TURN(46),
    BACK_LEFT_DRIVE(47);


    public int ID; 
    private CAN_IDs(int ID){
      this.ID=ID;
    }
  }

  public static class SwerveDriveConstants {

    

    
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
      public static final int FEEDER_MOTOR_CAN_ID            = 5;
      public static final int FLYWHEEL_MOTOR_CAN_ID          = 6;
      public static final int FLYWHEEL_follower_MOTOR_CAN_ID = 7;

      public static final class FeederSetPoints {
        public static final double FEED = 0.95;
      }

      public static final class FlywheelSetPoints {
        public static final double LAUNCH_RPM        = 5000;
        public static final double VELOCITY_TOLERANCE = 100;
      }
    }

}
