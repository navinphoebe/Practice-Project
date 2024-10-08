// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Field;

/** Add your docs here. */
public class MirroredSubzone {
    private FieldSubzone blueSubzone;
    private FieldSubzone redSubzone;

    public MirroredSubzone(String name, double southwestCornerX, double southwestCornerY, double width, double height, boolean flipZones) {
        if (flipZones) {
            blueSubzone = new FieldSubzone("Blue " + name, FieldMeasurements.convertToRedWidthMeters(southwestCornerX + width), southwestCornerY, width, height);
            redSubzone = new FieldSubzone("Red " + name, southwestCornerX, southwestCornerY, width, height);
        }
        else {        
            blueSubzone = new FieldSubzone("Blue " + name, southwestCornerX, southwestCornerY, width, height);
            redSubzone = new FieldSubzone("Red " + name, FieldMeasurements.convertToRedWidthMeters(southwestCornerX + width), southwestCornerY, width, height);
        }
    }

    public FieldSubzone getBlueSubzone() {
        return blueSubzone;
    }

    public FieldSubzone getRedSubzone() {
        return redSubzone;
    }
}