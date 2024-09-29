// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

/** Add your docs here. */
public class RavenDCMotorSim implements IRavenMotor {
    private DCMotorSim m_motor;
    public RavenDCMotorSim() {
        m_motor = new DCMotorSim(DCMotor.getNEO(1), 1.5, 0.004);
    }

    public RavenDCMotorSim(LinearSystem<N2, N1, N2> plant, DCMotor gearbox, double gearing) {
    m_motor = new DCMotorSim(plant, gearbox, gearing);
  }

 public RavenDCMotorSim(
      LinearSystem<N2, N1, N2> plant,
      DCMotor gearbox,
      double gearing,
      Matrix<N2, N1> measurementStdDevs) {
    m_motor = new DCMotorSim(plant, gearbox, gearing, measurementStdDevs);
  }
  
  public RavenDCMotorSim(DCMotor gearbox, double gearing, double jKgMetersSquared) {
    m_motor = new DCMotorSim(gearbox, gearing, jKgMetersSquared);
  }

  public RavenDCMotorSim(
      DCMotor gearbox, double gearing, double jKgMetersSquared, Matrix<N2, N1> measurementStdDevs) {
    m_motor = new DCMotorSim(gearbox, gearing, jKgMetersSquared, measurementStdDevs);
  }

  public void setState(double angularPositionRad, double angularVelocityRadPerSec) {
    m_motor.setState(angularPositionRad, angularVelocityRadPerSec);
  }

  public double getAngularPositionRad() {
    return m_motor.getAngularPositionRad();
  }

  public double getAngularPositionRotations() {
    return m_motor.getAngularPositionRotations();
  }

  public double getAngularVelocityRadPerSec() {
    return m_motor.getAngularVelocityRadPerSec();
  }

  @Override
  public double getVelocityRPM() {
    return m_motor.getAngularVelocityRPM();
  }

  @Override
  public double getVoltage() {
    return m_motor.getCurrentDrawAmps();
  }

  public double getCurrentDrawAmps() {
    return m_motor.getCurrentDrawAmps();
  }

  public void setVoltage(double volts) {
    m_motor.setInput(volts);
  }

  @Override
  public void setVelocity(double velocityRPM) {
    var velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    System.out.println("speed " + velocityRadPerSec);
    m_motor.setState(m_motor.getAngularPositionRad(), velocityRadPerSec);
  }

  @Override
  public void stop() {
    m_motor.setState(m_motor.getAngularPositionRad(), 0);
  }


    
}
