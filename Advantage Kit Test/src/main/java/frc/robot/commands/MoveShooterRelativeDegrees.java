// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveShooterRelativeDegrees extends Command {
  /** Creates a new MoveShooterRelativeDegrees. */
  private ShooterSubsystem m_shooter;
  private double m_addValue;
  private double m_changeValue;

  public MoveShooterRelativeDegrees(ShooterSubsystem shooter, double addValue) {
    m_shooter = shooter;
    m_addValue = addValue;
    m_changeValue = m_shooter.getAngle2() + m_addValue;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_changeValue = m_shooter.getAngle2() + m_addValue;
    m_shooter.setAngle2(m_changeValue);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
