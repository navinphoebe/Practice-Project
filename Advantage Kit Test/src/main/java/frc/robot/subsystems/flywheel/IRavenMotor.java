// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

/** Add your docs here. */
public interface IRavenMotor {
    public void setVelocity(double velocityRPM);

    public void stop();

    public double getVelocityRPM();

    public double getVoltage();
    
}
