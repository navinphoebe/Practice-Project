// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.DrivetrainState;
import frc.robot.Vision;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.DrivetrainSubsystemSim;

public class DrivetrainDefaultCommand extends Command {
  private Drivetrain m_drivetrain;
  private CommandXboxController m_driverController;
  private Vision m_vision;
  private double m_target;
  
  private ChassisSpeeds _chassisSpeeds = new ChassisSpeeds(0,0,0);
  /** Creates a new DrivetrainDefaultCommand. */
  public DrivetrainDefaultCommand(Drivetrain drivetrain, CommandXboxController driverController, Vision vision) {
    m_driverController = driverController;
    m_drivetrain = drivetrain;
    m_vision = vision;
    m_target = m_vision.getAlignDegrees(true);
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _chassisSpeeds = new ChassisSpeeds(0, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double controllerDirection = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 1 : -1;
    double x = m_driverController.getRawAxis(1) * controllerDirection;
    double y = m_driverController.getRawAxis(0) * controllerDirection;
    double r = m_driverController.getRawAxis(2) * -1;
    x = applyDeadband(x);
    y = applyDeadband(y);
    r = applyDeadband(r);
    double maxVelocity = m_drivetrain.getMaxVelocity();
    x = x * 1;
    y = y * 1;
    r = r * Constants.DRIVE_MAX_TURN_RADIANS_PER_SECOND;
    Rotation2d gyroRotation = m_drivetrain.getPose().getRotation();
    _chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, r, gyroRotation);

    m_drivetrain.drive(_chassisSpeeds);

    Logger.recordOutput("is target", m_vision.hasGoalTarget());
    if (Math.abs(m_vision.getAlignDegrees(true)) > 8 && RobotContainer.DRIVETRAIN_STATE == DrivetrainState.ROBOT_ALIGN) {
      m_target = m_vision.getAlignDegrees(true);
      Pose2d pose = m_drivetrain.getPose();
      Logger.recordOutput("align degrees", m_target);
      double sign = Math.copySign(1, m_target);
      m_drivetrain.resetPose(new Pose2d(pose.getX(), pose.getY(), new Rotation2d(Math.toRadians(pose.getRotation().getDegrees() - sign))));
      
      controllerDirection = DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 1 : -1;
      gyroRotation = m_drivetrain.getPose().getRotation();
    }
  }

  private double applyDeadband(double x) {
    if (Math.abs(x) > Constants.CONTROLLER_DEADBAND_VALUE) {
      return x;
    } else {
      return 0;
    }
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
