// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */
  private final EasySwerveModule m_frontLeft  = new EasySwerveModule(Constants.CAN_IDs.FRONT_LEFT_DRIVE.ID, Constants.CAN_IDs.FRONT_LEFT_TURN.ID, Constants.SwerveDriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET, true, true);
  private final EasySwerveModule m_backLeft   = new EasySwerveModule(0, 0, 0, false, false);
  private final EasySwerveModule m_frontRight = new EasySwerveModule(0, 0, 0, false, false);
  private final EasySwerveModule m_backRight  = new EasySwerveModule(0, 0, 0, false, false);

  

  public DriveSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
