// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Configs.IntakeSubsystemConfigs;
import frc.robot.Configs.LauncherSubsystemConfigs;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.LaunchAuto;
import frc.robot.commands.test;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_swerve = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final LauncherSubsystem m_launcher = new LauncherSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  private final CommandXboxController m_operatorController = 
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    m_swerve.setDefaultCommand(
      new RunCommand(
            () ->
                m_swerve.drive(
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(
                        m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    true),
            m_swerve).withName("Swerve Drive Default"));


    SmartDashboard.putData(m_intake);
    SmartDashboard.putData(m_launcher);
    
    SmartDashboard.putNumber("Bat Voltage", RobotController.getBatteryVoltage());
    
    SmartDashboard.putData("Intake", m_intake.runIntakeCommand().withName("Intake - Intaking"));
    SmartDashboard.putData("Extake", m_intake.runExtakeCommand().withName("Intake - Extaking"));
    
    SmartDashboard.putData("Feeder", m_launcher.runFeederCommand().withName("Shooter - Feeding and Shooting"));
    SmartDashboard.putData("Flywheel", m_launcher.runFlywheelCommand().withName("Shooter - Spinning up Flywheel"));
  }
  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

  

    //*******************************Driver Controller Button Assignments**********************************
  
    // Left Stick Button -> Set swerve to X
    m_driverController.leftStick().whileTrue(m_swerve.setXCommand());

    // Start Button -> Zero swerve heading
    m_driverController.start().onTrue(m_swerve.zeroHeadingCommand());

    // Right Trigger -> Run fuel intake in reverse
    m_driverController
      .rightTrigger(OIConstants.kTriggerButtonThreshold)
      .whileTrue(m_intake.runIntakeCommand());

    // Left Trigger -> Run fuel intake in reverse
    m_driverController
      .leftTrigger(OIConstants.kTriggerButtonThreshold)
      .whileTrue(m_intake.runExtakeCommand());


    // 
    m_driverController.y().onTrue(m_swerve.changeSpeed(Constants.SwerveDriveConstants.DriveSpeeds.FAST));
    m_driverController.b().onTrue(m_swerve.changeSpeed(Constants.SwerveDriveConstants.DriveSpeeds.MEDIUM));
    m_driverController.a().onTrue(m_swerve.changeSpeed(Constants.SwerveDriveConstants.DriveSpeeds.SLOW));


    //*******************************Operator Controller Button Assignments**********************************

    // X Button -> Run intake and run the shooter flywheel and feeder
    m_operatorController.x().toggleOnTrue(m_launcher.runLauncherCommand().alongWith(m_intake.runIntakeCommand()));

    //Right Bumper -> Intake
    m_operatorController.leftBumper().whileTrue(m_intake.runIntakeCommand());

    //Left Bumper -> Outtake 
    m_operatorController.rightBumper().whileTrue(m_intake.runExtakeCommand());
   
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new LaunchAuto(m_swerve,m_launcher).withTimeout(10);
    return new test(m_intake,m_launcher).withTimeout(10);


  }
}
