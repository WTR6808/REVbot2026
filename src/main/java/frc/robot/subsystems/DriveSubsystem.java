// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.PigeonTwo;

public class DriveSubsystem extends SubsystemBase {
  // Create EasySwerveModules
  private final EasySwerveModule m_frontLeft = new EasySwerveModule(
    CAN_IDs.FRONT_LEFT_DRIVE.ID,
    CAN_IDs.FRONT_LEFT_TURN.ID,
    SwerveDriveConstants.FRONT_LEFT_ANGULAR_OFFSET,
    SwerveDriveConstants.FRONT_LEFT_DRIVING_ON_BOTTOM,
    SwerveDriveConstants.FRONT_LEFT_TURNING_ON_BOTTOM);

private final EasySwerveModule m_frontRight = new EasySwerveModule(
    CAN_IDs.FRONT_RIGHT_DRIVE.ID,
    CAN_IDs.FRONT_RIGHT_TURN.ID,
    SwerveDriveConstants.FRONT_RIGHT_ANGULAR_OFFSET,
    SwerveDriveConstants.FRONT_RIGHT_DRIVING_ON_BOTTOM,
    SwerveDriveConstants.FRONT_RIGHT_TURNING_ON_BOTTOM);

private final EasySwerveModule m_rearLeft = new EasySwerveModule(
    CAN_IDs.BACK_LEFT_DRIVE.ID,
    CAN_IDs.BACK_LEFT_TURN.ID,
    SwerveDriveConstants.BACK_LEFT_ANGULAR_OFFSET,
    SwerveDriveConstants.BACK_LEFT_DRIVING_ON_BOTTOM,
    SwerveDriveConstants.BACK_LEFT_TURNING_ON_BOTTOM);

private final EasySwerveModule m_rearRight = new EasySwerveModule(
    CAN_IDs.BACK_RIGHT_DRIVE.ID,
    CAN_IDs.BACK_RIGHT_TURN.ID,
    SwerveDriveConstants.BACK_RIGHT_ANGULAR_OFFSET,
    SwerveDriveConstants.BACK_RIGHT_DRIVING_ON_BOTTOM,
    SwerveDriveConstants.BACK_RIGHT_TURNING_ON_BOTTOM);

private PigeonTwo m_pigeon = PigeonTwo.getInstance(); 


  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          SwerveDriveConstants.DRIVE_KINEMATICS,
          m_pigeon.getAngle(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
          });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {}

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        m_pigeon.getAngle(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_pigeon.getAngle(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * SwerveDriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double ySpeedDelivered = ySpeed * SwerveDriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double rotDelivered = rot * SwerveDriveConstants.MAX_ANGULAR_SPEED;

    var swerveModuleStates =
        SwerveDriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeedDelivered,
                    ySpeedDelivered,
                    rotDelivered,
                    m_pigeon.getAngle())
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, SwerveDriveConstants.MAX_ANGULAR_SPEED);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public Command setXCommand() {
    return this.run(
        () -> {
          m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
          m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
          m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        });
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, SwerveDriveConstants.MAX_SPEED_METERS_PER_SECOND);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeadingCommand() {
    return this.runOnce(() -> m_pigeon.reset());
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return m_pigeon.getAngle().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_pigeon.getRateY() * (SwerveDriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }
}
