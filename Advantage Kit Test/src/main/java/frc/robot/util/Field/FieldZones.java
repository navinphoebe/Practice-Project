// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.Field;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;

import java.awt.geom.Point2D;


public class FieldZones {
    public enum FieldMacroZone {
        NONE,
        LOADING_ZONE,
        COMMUNITY,
        CHARGE_STATION,
        NEUTRAL
    }

    private ArrayList<FieldZone> fieldZones = new ArrayList<FieldZone>();
    private FieldZone blueCommunity;
    private FieldZone redCommunity;
    private FieldZone blueLoadingZone;
    private FieldZone redLoadingZone;
    private FieldZone blueChargeStation;
    private FieldZone redChargeStation;
    private FieldZone neutralZone;
    
    // We'll include a none state in case odometry ever breaks by enough that no zone is returned.
    private FieldZone noneZone;

    // Field Zones for Rapid React
    public static final FieldSubzone blueInsideSafeZone = new FieldSubzone(
        "Blue Inside Safe Zone",
        2.5,
        4.25,
        1,
        1);

    public static final FieldSubzone blueOutsideSafeZone = new FieldSubzone(
        "Blue Outside Safe Zone",
        2.5,
        6.7,
        1,
        1);

    public static final FieldSubzone redInsideSafeZone = new FieldSubzone(
        "Red Inside Safe Zone",
        13,
        3,
        1,
        1);

    public static final FieldSubzone redOutsideSafeZone = new FieldSubzone(
        "Blue Outside Safe Zone",
        13,
        0.7,
        1,
        1);

    // Community.
    public static final MirroredSubzone COMMUNITY_NORTHERN_SECTION_MIRRORED_SUBZONE = new MirroredSubzone(
        "Community Northern Section",
        FieldMeasurements.COMMUNITY_NORTHERN_SECTION_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.COMMUNITY_NORTHERN_SECTION_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.COMMUNITY_NORTHERN_SECTION_WIDTH_METERS,
        FieldMeasurements.COMMUNITY_NORTHERN_SECTION_HEIGHT_METERS,
        false);
    
    public static final MirroredSubzone COMMUNITY_COOP_SECTION_MIRRORED_SUBZONE = new MirroredSubzone(
        "Community Coop Section",
        FieldMeasurements.COMMUNITY_COOP_SECTION_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.COMMUNITY_COOP_SECTION_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.COMMUNITY_COOP_SECTION_WIDTH_METERS,
        FieldMeasurements.COMMUNITY_COOP_SECTION_HEIGHT_METERS,
        false);

    public static final MirroredSubzone COMMUNITY_SOUTHERN_SECTION_MIRRORED_SUBZONE = new MirroredSubzone(
        "Community Southern Section",
        FieldMeasurements.COMMUNITY_SOUTHERN_SECTION_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.COMMUNITY_SOUTHERN_SECTION_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.COMMUNITY_SOUTHERN_SECTION_WIDTH_METERS,
        FieldMeasurements.COMMUNITY_SOUTHERN_SECTION_HEIGHT_METERS,
        false);

    public static final MirroredSubzone COMMUNITY_CHARGE_STATION_MIRRORED_SUBZONE = new MirroredSubzone(
        "Community Charge Station",
        FieldMeasurements.COMMUNITY_CHARGE_STATION_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.COMMUNITY_CHARGE_STATION_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.COMMUNITY_CHARGE_STATION_WIDTH_METERS,
        FieldMeasurements.COMMUNITY_CHARGE_STATION_HEIGHT_METERS,
        false);
    
    // Loading zone.
    public static final MirroredSubzone LOADING_ZONE_WIDE_SECTION_MIRRORED_SUBZONE = new MirroredSubzone(
        "Loading Zone Wide Section",
        FieldMeasurements.LOADING_ZONE_WIDE_AREA_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.LOADING_ZONE_WIDE_AREA_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.LOADING_ZONE_WIDE_AREA_WIDTH_METERS,
        FieldMeasurements.LOADING_ZONE_WIDE_AREA_HEIGHT_METERS,
        true);

    public static final MirroredSubzone LOADING_ZONE_NARROW_SECTION_MIRRORED_SUBZONE = new MirroredSubzone(
        "Loading Zone Narrow Section",
        FieldMeasurements.LOADING_ZONE_NARROW_AREA_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.LOADING_ZONE_NARROW_AREA_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.LOADING_ZONE_NARROW_AREA_WIDTH_METERS,
        FieldMeasurements.LOADING_ZONE_NARROW_AREA_HEIGHT_METERS,
        true);
    
    // Neutral zone. Unlike the other zones, the NZ is NOT mirrored - it's just one large FieldZone.
    // However the width coordinates are defined by halves so need to be doubled.
    public static final FieldSubzone neutralZoneNarrowArea = new FieldSubzone(
        "Neutral Zone Narrow Area",
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_WIDTH_METERS * 2,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NARROW_AREA_HEIGHT_METERS);
    
    public static final FieldSubzone neutralZoneNorthernArea = new FieldSubzone(
        "Neutral Zone Northern Area",
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_WIDTH_METERS * 2,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_NORTHERN_AREA_HEIGHT_METERS);  
        
    public static final FieldSubzone neutralZoneSouthernArea = new FieldSubzone(
        "Neutral Zone Southern Area",
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_SOUTHWEST_CORNER_X_METERS,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_SOUTHWEST_CORNER_Y_METERS,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_WIDTH_METERS * 2,
        FieldMeasurements.NEUTRAL_ZONE_HALF_FIELD_SOUTHERN_AREA_HEIGHT_METERS);
    
    public static final FieldSubzone noneSubzone = new FieldSubzone(
        "None",
        -1,
        -1,
        0,
        0);

    
    
    public FieldZones() {
        // Communities.
        MirroredFieldZone communityMirroredFieldZone = new MirroredFieldZone(FieldMacroZone.COMMUNITY, "Community", COMMUNITY_NORTHERN_SECTION_MIRRORED_SUBZONE);
            communityMirroredFieldZone.addMirroredSubzone(COMMUNITY_COOP_SECTION_MIRRORED_SUBZONE);
            communityMirroredFieldZone.addMirroredSubzone(COMMUNITY_SOUTHERN_SECTION_MIRRORED_SUBZONE);
            communityMirroredFieldZone.addMirroredSubzone(COMMUNITY_CHARGE_STATION_MIRRORED_SUBZONE);

        blueCommunity = communityMirroredFieldZone.getBlueFieldZone();
        redCommunity = communityMirroredFieldZone.getRedFieldZone();
        blueCommunity.generateBoundingBox();
        redCommunity.generateBoundingBox();
        fieldZones.add(blueCommunity);
        fieldZones.add(redCommunity);

        // Loading zones.
        MirroredFieldZone loadingZoneMirroredFieldZone = new MirroredFieldZone(FieldMacroZone.LOADING_ZONE, "Loading Zone", LOADING_ZONE_WIDE_SECTION_MIRRORED_SUBZONE);
            loadingZoneMirroredFieldZone.addMirroredSubzone(LOADING_ZONE_NARROW_SECTION_MIRRORED_SUBZONE);

        blueLoadingZone = loadingZoneMirroredFieldZone.getBlueFieldZone();
        redLoadingZone = loadingZoneMirroredFieldZone.getRedFieldZone();
        blueLoadingZone.generateBoundingBox();
        redLoadingZone.generateBoundingBox();
        fieldZones.add(blueLoadingZone);
        fieldZones.add(redLoadingZone);

        // Neutral zone. Changed Alliance Invalid to Blue
        neutralZone = new FieldZone(Alliance.Blue, FieldMacroZone.NEUTRAL, "Neutral Zone", neutralZoneNarrowArea);
        neutralZone.addSubzone(neutralZoneNorthernArea);
        neutralZone.addSubzone(neutralZoneSouthernArea);
        neutralZone.generateBoundingBox();
        neutralZoneNarrowArea.setFieldZone(neutralZone);
        neutralZoneNorthernArea.setFieldZone(neutralZone);
        neutralZoneSouthernArea.setFieldZone(neutralZone);

        fieldZones.add(neutralZone);

        // The charge stations are a subzone of their respective communities, but also their own zones.
        MirroredFieldZone chargeStationMirroredFieldZone = new MirroredFieldZone(FieldMacroZone.CHARGE_STATION, "Charge Station", COMMUNITY_CHARGE_STATION_MIRRORED_SUBZONE);
        blueChargeStation = chargeStationMirroredFieldZone.getBlueFieldZone();
        redChargeStation = chargeStationMirroredFieldZone.getRedFieldZone();
        blueChargeStation.generateBoundingBox();
        redChargeStation.generateBoundingBox();
        fieldZones.add(blueChargeStation);
        fieldZones.add(redChargeStation);

        // Create the none zone. Changed Invalid to blue
        noneZone = new FieldZone(Alliance.Blue, FieldMacroZone.NONE, "None", noneSubzone);
        noneZone.generateBoundingBox();
        noneSubzone.setFieldZone(noneZone);
        fieldZones.add(noneZone);
    }

    public FieldSubzone getPointFieldZone(Point2D point) {
        // If odometry is working correctly we'll always find a zone, but if it's not, we need a default.
        FieldSubzone robotFieldSubzone = noneSubzone;

            for (int i = 0; i < fieldZones.size(); i++) {
                if (fieldZones.get(i).containsPoint(point)) {
                    robotFieldSubzone = fieldZones.get(i).subzonesContainPoint(point);

                    if (robotFieldSubzone != null) {
                        break;
                    }
            }
        }
        
        if (robotFieldSubzone == null) {
            robotFieldSubzone = FieldZones.noneSubzone;
        }

        return robotFieldSubzone;
    }

    public void output() {
        //System.out.println("Zones:");

        for (FieldZone zone : fieldZones) {
            zone.outputSubzones();
        }

       // System.out.println();
        //System.out.println();
    }

    public FieldZone getNoneZone() {
        return noneZone;
    }

    public ArrayList<FieldZone> getFieldZones() {
        return fieldZones;
        // ArrayList<FieldZone> rapidReactFieldZones = new ArrayList<>();
        // rapidReactFieldZones.add(new FieldZone(Alliance.Blue, FieldMacroZone.NONE, "Blue Inside Safe Zone", blueInsideSafeZone));
        // rapidReactFieldZones.add(new FieldZone(Alliance.Blue, FieldMacroZone.NONE, "Blue Outside Safe Zone", blueOutsideSafeZone));
        // rapidReactFieldZones.add(new FieldZone(Alliance.Red, FieldMacroZone.NONE, "Red Inside Safe Zone", redInsideSafeZone));
        // rapidReactFieldZones.add(new FieldZone(Alliance.Red, FieldMacroZone.NONE, "Red Outside Safe Zone", redOutsideSafeZone));
        // return rapidReactFieldZones;
    }
}