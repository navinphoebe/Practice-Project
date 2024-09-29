// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.flywheel.FlywheelSubystem;

public class StartFlywheelCommand extends Command {
  /** Creates a new StartFlywheelCommand. */
  private FlywheelSubystem m_flywheel;
  public StartFlywheelCommand(FlywheelSubystem flywheel) {
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
    System.out.println("execute command flywheel");
    m_flywheel.setVelocity(Constants.TARGET_RPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_flywheel.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
