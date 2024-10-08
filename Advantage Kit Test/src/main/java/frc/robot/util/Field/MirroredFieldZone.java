// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Field;

/** Add your docs here. */
import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.Field.FieldZones.FieldMacroZone;

public class MirroredFieldZone {
    private ArrayList<MirroredSubzone> mirroredSubzones = new ArrayList<MirroredSubzone>();
    private FieldZone blueFieldZone;
    private FieldZone redFieldZone;

    public MirroredFieldZone(FieldMacroZone macroZone, String zoneName, MirroredSubzone initialMirroredSubzone) {
        blueFieldZone = new FieldZone(Alliance.Blue, macroZone, "Blue " + zoneName, initialMirroredSubzone.getBlueSubzone());
        initialMirroredSubzone.getBlueSubzone().setFieldZone(blueFieldZone);

        redFieldZone = new FieldZone(Alliance.Red, macroZone, "Red " + zoneName, initialMirroredSubzone.getRedSubzone());
        initialMirroredSubzone.getRedSubzone().setFieldZone(redFieldZone);
    }

    // Create a field zone, which must be initialized with at least one subzone.
    public void addMirroredSubzone(MirroredSubzone mirroredSubzone) {
        mirroredSubzones.add(mirroredSubzone);
        
        blueFieldZone.addSubzone(mirroredSubzone.getBlueSubzone());
        mirroredSubzone.getBlueSubzone().setFieldZone(blueFieldZone);

        redFieldZone.addSubzone(mirroredSubzone.getRedSubzone());
        mirroredSubzone.getRedSubzone().setFieldZone(redFieldZone);
    }

    public FieldZone getBlueFieldZone() {
        return blueFieldZone;
    }

    public FieldZone getRedFieldZone() {
        return redFieldZone;
    }
}