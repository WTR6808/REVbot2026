// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.ModuleConstants;

import com.revrobotics.spark.config.SparkFlexConfig;

/** Add your docs here. */
public final class Configs {
    private static final double nominalVoltage = 12.0;

    public static final class EasySwerveModule {
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkFlexConfig turningConfig = new SparkFlexConfig();
        static{
         // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = Constants.ModuleConstants.WHEEL_DIAMETER_METERS * Math.PI
             / Constants.ModuleConstants.DRIVING_MOTOR_REDUCTION;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_RPS;

            drivingConfig
              .idleMode(IdleMode.kBrake)
              .smartCurrentLimit(60);
            drivingConfig
                .encoder
                    .positionConversionFactor(drivingFactor)
                    .velocityConversionFactor(drivingFactor / 60.0);
            drivingConfig
                .closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                     // These are example gains you may need to adjust them for your own robot!
                    .pid(0.04, 0, 0)
                    .outputRange(-1, 1)
                    .feedForward 
                        .kV(drivingVelocityFeedForward);
            
            turningConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60);
            turningConfig 
                .absoluteEncoder
                 // Do not invert the turning encoder, since the output shaft rotates in the same
                // direction as the steering motor in the EasySwerve Module.
                    .inverted(false)
                    .positionConversionFactor(turningFactor)
                    .velocityConversionFactor(turningFactor / 60.0)
                     // These apply to REV Through Bore Encoder V2 (for V1, set them both to 1.0):
                    .startPulseUs(3.88443797)
                    .endPulseUs(1.94221899);
            turningConfig
                .closedLoop 
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to adjust them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                   // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);

        }

    }

public static final class IntakeSubsystemConfigs {
    public static final SparkFlexConfig intakeConfig = new SparkFlexConfig();
    public static final SparkFlexConfig conveyorConfig = new SparkFlexConfig();

    static {
        intakeConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .openLoopRampRate(0.5)
            .smartCurrentLimit(40);

        conveyorConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .openLoopRampRate(0.5)
            .smartCurrentLimit(40);
    }
}

public static final class LauncherSubsystemConfigs {
    public static final SparkFlexConfig flywheelConfig = new SparkFlexConfig();
    public static final SparkFlexConfig flywheelfollowerConfig = new SparkFlexConfig();
    public static final SparkFlexConfig feederConfig = new SparkFlexConfig();

    static {
        flywheelConfig.inverted(true).idleMode(IdleMode.kCoast).closedLoopRampRate(1.0).openLoopRampRate(1.0).smartCurrentLimit(80);
        flywheelConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).p(.0002).outputRange(-1, 1);
        flywheelConfig.closedLoop.maxMotion.cruiseVelocity(5000).maxAcceleration(10000).allowedProfileError(1);
        flywheelConfig.closedLoop.feedForward.kV(nominalVoltage/Constants.NeoMotorConstants.VORTEX_KV);
        flywheelfollowerConfig.apply(feederConfig).follow(Constants.LauncherSubsystemConstants.FLYWHEEL_MOTOR_CAN_ID, true);
        feederConfig.inverted(false).idleMode(IdleMode.kCoast).openLoopRampRate(1.0).smartCurrentLimit(60);

    }
}




}
