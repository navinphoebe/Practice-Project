// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Vision;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.ShooterSubsystem;

public class TurnToAutoAlign extends Command {
  /** Creates a new MoveArmAndShooterToPosition. */
  private Drivetrain m_drivetrain;
  private double m_target;
  private Vision m_vision;
  private double m_angle;
  public TurnToAutoAlign(Drivetrain drivetrain, Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_target = vision.getAlignDegrees();
    m_vision = vision;

    addRequirements(m_drivetrain);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_vision.hasGoalTarget()) {
      Pose2d pose = m_drivetrain.getPose();
      m_target = m_vision.getAlignDegrees();
      Logger.recordOutput("align degrees", m_target);
      double sign = Math.copySign(1, m_target);
      m_drivetrain.resetPose(new Pose2d(pose.getX(), pose.getY(), new Rotation2d(Math.toRadians(pose.getRotation().getDegrees() - sign))));
  
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_vision.getAlignDegrees()) < 5;
  }
}
