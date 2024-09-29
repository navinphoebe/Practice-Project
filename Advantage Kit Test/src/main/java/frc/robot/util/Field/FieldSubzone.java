// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Field;

import java.awt.geom.Rectangle2D;

import edu.wpi.first.math.geometry.Pose2d;

import java.awt.geom.Point2D;

/** Add your docs here. */
public class FieldSubzone {
    private FieldZone fieldZone;
    private String name;
    private Point2D southwestCorner;
    private Point2D northeastCorner;
    private Rectangle2D boundingBox;

    public FieldSubzone(String name, double southwestCornerX, double southwestCornerY, double width, double height) {
        this.name = name;

        Point2D southwestCorner = new Point2D.Double(southwestCornerX, southwestCornerY);
        Point2D northeastCorner = new Point2D.Double(southwestCornerX + width, southwestCornerY + height);

        this.southwestCorner = southwestCorner;
        this.northeastCorner = northeastCorner;
        
        this.boundingBox = new Rectangle2D.Double(southwestCornerX, southwestCornerY, width, height);
    }

    public String getName() {
        return name;
    }

    public Rectangle2D getBoundingBox() {
        return boundingBox;
    }

    public Point2D getSouthwestCorner() {
        return southwestCorner;
    }

    public Point2D getNortheastCorner() {
        return northeastCorner;
    }

    public void setFieldZone(FieldZone fieldZone) {
        this.fieldZone = fieldZone;
    }

    public boolean containsPoint(Point2D point) {
        return boundingBox.contains(point);
    }

    public FieldZone getFieldZone() {
        return fieldZone;
    }

    public void output() {
       // System.out.println("Zone: " + name);
       // System.out.println("SW Corner: " + southwestCorner.getX() + ", " + southwestCorner.getY());
       // System.out.println("NE Corner: " + northeastCorner.getX() + ", " + northeastCorner.getY());

       // System.out.println();
       // System.out.println();
    }
}
