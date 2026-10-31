// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.LauncherSubsystemConstants;
import frc.robot.Constants.LauncherSubsystemConstants.FeederSetPoints;
import frc.robot.Constants.LauncherSubsystemConstants.FlywheelSetPoints;
import frc.robot.utilities.Configs;

public class LauncherSubsystem extends SubsystemBase {
  /** Creates a new Launcher. */



private SparkFlex flywheelMotor = new SparkFlex(CAN_IDs.FLYWHEEL.ID, MotorType.kBrushless); 
private SparkClosedLoopController flywheelController = flywheelMotor.getClosedLoopController();
private RelativeEncoder flywheelEncoder = flywheelMotor.getEncoder();

private SparkFlex flywheelfollowerMotor = new SparkFlex(CAN_IDs.FLYWHEEL_FOLLOWER.ID, MotorType.kBrushless);

private SparkFlex feederMotor = new SparkFlex(CAN_IDs.FEEDER.ID, MotorType.kBrushless);

private double flywheelTargetVelocity = 0;




  public LauncherSubsystem() {
    flywheelMotor.configure(Configs.LauncherSubsystemConfigs.flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelfollowerMotor.configure(Configs.LauncherSubsystemConfigs.flywheelfollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    feederMotor.configure(Configs.LauncherSubsystemConfigs.feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    flywheelEncoder.setPosition(0);
  }
  
  private boolean isFlywheelAt(double velocity){
    return MathUtil.isNear(flywheelEncoder.getVelocity(), velocity, FlywheelSetPoints.VELOCITY_TOLERANCE);
  }

public final Trigger isFlywheelSpinning = new Trigger(()->isFlywheelAt(5000)|| flywheelEncoder.getVelocity()>5000);


public final Trigger isFlywheelSpinningBackwards = new Trigger(()->isFlywheelAt(-5000)|| flywheelEncoder.getVelocity()<-5000);

public final Trigger isFlywheelStopped = new Trigger(()->isFlywheelAt(0));

private void setFlywheelVelocity(double velocity){
  flywheelController.setSetpoint(velocity, ControlType.kMAXMotionVelocityControl);
  flywheelTargetVelocity = velocity;
}

private void setFeederPower(double power){
  feederMotor.set(power);
}

public Command runFlywheelCommand(){
  return this.startEnd(()->{
    this.setFlywheelVelocity(FlywheelSetPoints.LAUNCH_RPM);
  },()->{
    this.setFlywheelVelocity(0);
  }).withName("spinning the flywheel");
  }

  public Command runFeederCommand(){
  return this.startEnd(()->{
    this.setFlywheelVelocity(FlywheelSetPoints.LAUNCH_RPM);
    this.setFeederPower(FeederSetPoints.FEED);
  },()->{
    this.setFlywheelVelocity(0);
    this.setFeederPower(0);
  }).withName("spinning the flywheel");
  }

  public Command runLauncherCommand(){
    return this.startEnd(
      ()->this.setFlywheelVelocity(FlywheelSetPoints.LAUNCH_RPM),
      ()->flywheelMotor.stopMotor()
    ).until(isFlywheelSpinning).andThen(
      this.startEnd(
        ()->{
          this.setFlywheelVelocity(FlywheelSetPoints.LAUNCH_RPM);
          this.setFeederPower(FeederSetPoints.FEED);
        }, ()->{
          flywheelMotor.stopMotor();
          feederMotor.stopMotor();
        })
    ).withName("Launching");
  }

  public Command runFeederReverse(){
    return this.startEnd(
      ()-> this.setFeederPower(FeederSetPoints.FEED_REVERSE), 
      ()-> this.setFeederPower(0.0));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Launcher | Feeder | Applied Output", feederMotor.getAppliedOutput());
    SmartDashboard.putNumber("Launcher | Flywheel | Applied Output", flywheelMotor.getAppliedOutput());
    SmartDashboard.putNumber("Launcher | Flywheel | Current", flywheelMotor.getOutputCurrent());
    SmartDashboard.putNumber("Launcher | Flywheel follower| Applied Output", flywheelfollowerMotor.getAppliedOutput());
    SmartDashboard.putNumber("Launcher | Flywheel follower| Current", flywheelfollowerMotor.getOutputCurrent());
    
    SmartDashboard.putNumber("Launcher | Flywheel | Target Velocity", flywheelTargetVelocity);
    SmartDashboard.putNumber("Launcher | Flywheel | Actual Velocity", flywheelEncoder.getVelocity());

    SmartDashboard.putBoolean("Is Flywheel Spinning", isFlywheelSpinning.getAsBoolean());
    SmartDashboard.putBoolean("Is Flywheel Stopped", isFlywheelStopped.getAsBoolean());
  }
}
