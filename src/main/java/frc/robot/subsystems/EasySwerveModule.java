// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class EasySwerveModule extends SubsystemBase {
  private final SparkFlex m_drivingFlex;
  private final SparkFlex m_turningFlex;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private SparkClosedLoopController m_drivingClosedLoopController;
  private SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0.0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());


  /** Creates a new EasySwerveModule. */
  public EasySwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, 
                            boolean drivingMotorOnBottom, boolean turningMotorOnBottom) {

   m_drivingFlex = new SparkFlex(drivingCANId, MotorType.kBrushless);
   m_turningFlex = new SparkFlex(turningCANId, MotorType.kBrushless);

   m_drivingEncoder = m_drivingFlex.getEncoder();
   m_turningEncoder = m_turningFlex.getAbsoluteEncoder();

   m_drivingClosedLoopController = m_drivingFlex.getClosedLoopController();
   m_turningClosedLoopController = m_turningFlex.getClosedLoopController();


   //configs
   SparkFlexConfig drivingConfig = Configs.EasySwerveModule.drivingConfig;
   drivingConfig.inverted(drivingMotorOnBottom);
   m_drivingFlex.configure(drivingConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
   SparkFlexConfig turningConfig = Configs.EasySwerveModule.turningConfig;
   turningConfig.inverted(!turningMotorOnBottom);
   m_turningFlex.configure(turningConfig, com.revrobotics.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

   m_chassisAngularOffset = chassisAngularOffset;
   m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
   m_drivingEncoder.setPosition(0);
  
  }

  public SwerveModuleState getState(){

    return new SwerveModuleState(m_drivingEncoder.getVelocity(), 
           new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_drivingEncoder.getPosition(), 
           new Rotation2d(m_turningEncoder.getPosition() - m_chassisAngularOffset));
  }
  
public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
