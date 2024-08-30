// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FlywheelSubystemSim;

public class SetMotorSpeed extends Command {
  /** Creates a new SetMotorSpeed. */
  private FlywheelSubystemSim m_flywheel;
  private double m_speed;
  public SetMotorSpeed(FlywheelSubystemSim flywheel, double speed) {
    m_speed = speed;
    m_flywheel = flywheel;
    addRequirements(m_flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_flywheel.setPowerManually(m_speed);
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
