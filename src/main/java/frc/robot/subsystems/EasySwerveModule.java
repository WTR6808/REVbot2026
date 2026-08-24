// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;

public class EasySwerveModule extends SubsystemBase {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private SparkClosedLoopController m_drivingClosedLoopController;
  private SparkClosedLoopController m_turningClosedLoopController;

  private double m_chassisAngularOffset = 0.0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());


  /** Creates a new EasySwerveModule. */
  public EasySwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset, 
                            boolean drivingMotorOnBottom, boolean turningMotorOnBottom) {

   m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
   m_turningSpark = new SparkMax(turningCANId, MotorType.kBrushless);

   m_drivingEncoder = m_drivingSpark.getEncoder();
   m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

   m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
   m_turningClosedLoopController = m_turningSpark.getClosedLoopController();


   //configs
   SparkMaxConfig drivingConfig = Configs.EasySwerveModule.drivingConfig;
   drivingConfig.inverted(drivingMotorOnBottom);
   m_drivingSpark.configure(drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  
   SparkMaxConfig turningConfig = Configs.EasySwerveModule.turningConfig;
   turningConfig.inverted(!turningMotorOnBottom);
   m_turningSpark.configure(turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
