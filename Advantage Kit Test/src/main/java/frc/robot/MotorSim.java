// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class MotorSim {

    private double m_speed = 0;

    public MotorSim(){

    }

    public void set(double speed) {
        m_speed = speed;
    }

    public double getSpeed() {
        return m_speed;
    }
}
