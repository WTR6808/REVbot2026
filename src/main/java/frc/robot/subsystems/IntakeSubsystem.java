// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  private SparkFlex intakeMotor = 
      new SparkFlex(Constants.IntakeSubsytemConstants.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);

   private SparkFlex conveyorMotor = 
      new SparkFlex(Constants.IntakeSubsytemConstants.CONVEYOR_MOTOR_CAN_ID, MotorType.kBrushless);

  public IntakeSubsystem() {

    intakeMotor.configure(
      Configs.IntakeSubsystem.intakeConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    conveyorMotor.configure(
      Configs.IntakeSubsystem.conveyorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    System.out.println("---> IntakeSubsystem initialized");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
