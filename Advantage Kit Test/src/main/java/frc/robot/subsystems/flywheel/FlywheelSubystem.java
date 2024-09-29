// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.flywheel;

import edu.wpi.first.wpilibj2.command.Subsystem;

public interface FlywheelSubystem extends Subsystem {

    public void setVelocity(double velocityRPM);

    public void stop();
}
