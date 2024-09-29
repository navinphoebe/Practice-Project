// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.DrivetrainState;
import frc.robot.RobotContainer.PickupState;
import frc.robot.Vision;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterDefaultCommand extends Command {
  /** Creates a new AutoAdjustWithAprilTags. */
  private ShooterSubsystem m_shooter;
  private Vision m_vision;
  private double targetAngle;
  public ShooterDefaultCommand(Vision vision, ShooterSubsystem shooter) {
    m_shooter = shooter;
    m_vision = vision;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("init auto tag");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /* if (RobotContainer.DRIVETRAIN_STATE == DrivetrainState.ROBOT_ALIGN && RobotContainer.PICKUP_STATE == PickupState.HAS_NOTE) {
      double ty = m_vision.getGoalDistance(true);
      targetAngle = m_shooter.getShooterAngleMapDown(ty);
      Logger.recordOutput("default", targetAngle);
      if ((Math.abs((targetAngle - m_shooter.getAngle2()) % 360)) > 2) {
      double sign = calculateNegativeOrPositive();
      m_shooter.changeAngle2(sign);
      }
    } */
    targetAngle = m_shooter.getTargetAngle();
    Logger.recordOutput("default", targetAngle);
      if ((Math.abs((targetAngle - m_shooter.getAngle2()) % 360)) > 2) {
      double sign = calculateNegativeOrPositive();
      m_shooter.changeAngle2(sign);
      }
  }

  public double calculateNegativeOrPositive() {
    double m_shooterTarget = targetAngle % 360;
    m_shooterTarget += 360;
    m_shooterTarget = m_shooterTarget % 360;

    double m_shooterAngle = m_shooter.getAngle2() % 360;
    m_shooterAngle += 360;
    m_shooterAngle = m_shooterAngle % 360;
    m_shooter.setAngle2(m_shooterAngle);
    boolean greater = (m_shooterTarget - m_shooterAngle > 0 && m_shooterTarget - m_shooterAngle < 180) || m_shooterTarget - m_shooterAngle < -180;
    boolean equal = m_shooterTarget - m_shooterAngle == 0;
    if (greater ) {
      return 1;
    } else if (equal) {
      return 0;
    } else {
      return -1;
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
