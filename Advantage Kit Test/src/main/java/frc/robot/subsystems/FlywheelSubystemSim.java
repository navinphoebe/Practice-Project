// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotorSim;

public class FlywheelSubystemSim extends SubsystemBase {
  private final MotorSim m_leftMotor = new MotorSim();
  private final MotorSim m_rightMotor = new MotorSim();
  /** Creates a new FlywheelSubystemSim. */
  public FlywheelSubystemSim() {

  }

  public void setPowerManually(double speedRadiansPerSecond) {
    m_leftMotor.set(speedRadiansPerSecond);
    m_rightMotor.set(speedRadiansPerSecond);
}

public void stopPower() {
    m_leftMotor.set(0);
    m_rightMotor.set(0);
}

  @AutoLogOutput
  public double getLeftMotorRadiansPerSecond() {
    return m_leftMotor.getSpeed();
  }

  @AutoLogOutput
  public double getRightMotorRadiansPerSecond() {
    return m_rightMotor.getSpeed();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(m_leftMotor.getSpeed());
  }
}
