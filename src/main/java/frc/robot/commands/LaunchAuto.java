// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.ejml.dense.block.MatrixOps_DDRB;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LaunchAuto extends Command {

  private IntakeSubsystem m_intake;
  private LauncherSubsystem m_launcher;
  /** Creates a new LaunchAuto. */
  public LaunchAuto(IntakeSubsystem Intake, LauncherSubsystem Launch) {
    // Use addRequirements() here to declare subsystem dependencies.

    m_intake = Intake;
    m_launcher = Launch;

    addRequirements(m_intake, m_launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
