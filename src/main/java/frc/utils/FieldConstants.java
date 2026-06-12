// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
// import lombok.Getter;
// import lombok.RequiredArgsConstructor;
import java.nio.file.Path;

/**
 * Contains information for location of field element and other useful reference
 * points.
 *
 * <p>
 * NOTE: All constants are defined relative to the field coordinate system, and
 * from the
 * perspective of the blue alliance station
 */
public class FieldConstants {
    public static final AprilTagLayout layout = AprilTagLayout.OFFICIAL_ANDYMARK;
    //public static final AprilTagFields field = AprilTagFields.k2026RebuiltAndymark;
    // public static final FieldType fieldType = FieldType.WELDED;

    // AprilTag related constants
    public static final int aprilTagCount = layout.getConstLayout().getTags().size();
    public static final double aprilTagWidth = Units.inchesToMeters(6.5);

    // Field dimensions
    public static final double fieldLength = layout.getConstLayout().getFieldLength();
    public static final double fieldWidth = layout.getConstLayout().getFieldWidth();

    /**
     * Officially defined and relevant vertical lines found on the field (defined by
     * X-axis offset)
     */
    public static class LinesVertical {
        public static final double center = fieldLength / 2.0;
        public static final double starting = layout.getConstLayout().getTagPose(26).get().getX();
        public static final double allianceZone = starting;
        public static final double hubCenter = layout.getConstLayout().getTagPose(26).get().getX()
                + Hub.width / 2.0;
        public static final double neutralZoneNear = center - Units.inchesToMeters(120);
        public static final double neutralZoneFar = center + Units.inchesToMeters(120);
        public static final double oppHubCenter = layout.getConstLayout().getTagPose(4).get().getX()
                + Hub.width / 2.0;
        public static final double oppAllianceZone = layout.getConstLayout().getTagPose(10).get()
                .getX();

        public static final double trenchCenter = (allianceZone + neutralZoneNear) / 2.0;
    }

    /**
     * Officially defined and relevant horizontal lines found on the field (defined
     * by Y-axis offset)
     *
     * <p>
     * NOTE: The field element start and end are always left to right from the
     * perspective of the
     * alliance station
     */
    public static class LinesHorizontal {

        public static final double center = fieldWidth / 2.0;

        // Right of hub
        public static final double rightBumpStart = Hub.nearRightCorner.getY();
        public static final double rightBumpEnd = rightBumpStart - RightBump.width;
        public static final double rightBumpMiddle = (rightBumpStart + rightBumpEnd) / 2.0;
        public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
        public static final double rightTrenchOpenEnd = 0;
        public static final double rightTrenchMiddle = rightTrenchOpenStart / 2.0;
        public static final double rightMiddle = (rightBumpEnd + rightTrenchOpenStart) / 2.0; // in between trench & bump

        // Left of hub
        public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
        public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
        public static final double leftBumpMiddle = (leftBumpEnd + leftBumpStart) / 2.0;
        public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
        public static final double leftTrenchOpenStart = fieldWidth;
        public static final double leftTrenchMiddle = (leftTrenchOpenEnd + leftTrenchOpenStart) / 2.0;
        public static final double leftMiddle = (leftBumpStart + leftTrenchOpenEnd) / 2.0; // in between trench & bump
    }

    /** Hub related constants */
    public static class Hub {

        // Dimensions
        public static final double width = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(72.0); // includes the catcher at the top
        public static final double innerWidth = Units.inchesToMeters(41.7);
        public static final double innerHeight = Units.inchesToMeters(56.5);

        // Relevant reference points on alliance side
        public static final Translation3d topCenterPoint = new Translation3d(
                layout.getConstLayout().getTagPose(26).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                height);
        public static final Translation3d innerCenterPoint = new Translation3d(
                layout.getConstLayout().getTagPose(26).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                innerHeight);

        public static final Translation2d nearLeftCorner = new Translation2d(topCenterPoint.getX() - width / 2.0,
                fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d nearRightCorner = new Translation2d(topCenterPoint.getX() - width / 2.0,
                fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d farLeftCorner = new Translation2d(topCenterPoint.getX() + width / 2.0,
                fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d farRightCorner = new Translation2d(topCenterPoint.getX() + width / 2.0,
                fieldWidth / 2.0 - width / 2.0);

        // Relevant reference points on the opposite side
        public static final Translation3d oppTopCenterPoint = new Translation3d(
                layout.getConstLayout().getTagPose(4).get().getX() + width / 2.0,
                fieldWidth / 2.0,
                height);
        public static final Translation2d oppNearLeftCorner = new Translation2d(oppTopCenterPoint.getX() - width / 2.0,
                fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d oppNearRightCorner = new Translation2d(oppTopCenterPoint.getX() - width / 2.0,
                fieldWidth / 2.0 - width / 2.0);
        public static final Translation2d oppFarLeftCorner = new Translation2d(oppTopCenterPoint.getX() + width / 2.0,
                fieldWidth / 2.0 + width / 2.0);
        public static final Translation2d oppFarRightCorner = new Translation2d(oppTopCenterPoint.getX() + width / 2.0,
                fieldWidth / 2.0 - width / 2.0);

        // Hub faces
        public static final Pose2d nearFace = layout.getConstLayout().getTagPose(26).get().toPose2d();
        public static final Pose2d farFace = layout.getConstLayout().getTagPose(20).get().toPose2d();
        public static final Pose2d rightFace = layout.getConstLayout().getTagPose(18).get().toPose2d();
        public static final Pose2d leftFace = layout.getConstLayout().getTagPose(21).get().toPose2d();
    }

    /** Left Bump related constants */
    public static class LeftBump {

        // Dimensions
        public static final double width = Units.inchesToMeters(73.0);
        public static final double height = Units.inchesToMeters(6.513);
        public static final double depth = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side
        public static final Translation2d nearLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
        public static final Translation2d farLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d farRightCorner = Hub.farLeftCorner;

        // Relevant reference points on opposing side
        public static final Translation2d oppNearLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
        public static final Translation2d oppFarLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Right Bump related constants */
    public static class RightBump {
        // Dimensions
        public static final double width = Units.inchesToMeters(73.0);
        public static final double height = Units.inchesToMeters(6.513);
        public static final double depth = Units.inchesToMeters(44.4);

        // Relevant reference points on alliance side
        public static final Translation2d nearLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
        public static final Translation2d farLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d farRightCorner = Hub.farLeftCorner;

        // Relevant reference points on opposing side
        public static final Translation2d oppNearLeftCorner = new Translation2d(LinesVertical.hubCenter + width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
        public static final Translation2d oppFarLeftCorner = new Translation2d(LinesVertical.hubCenter - width / 2,
                Units.inchesToMeters(255));
        public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
    }

    /** Left Trench related constants */
    public static class LeftTrench {
        // Dimensions
        public static final double width = Units.inchesToMeters(65.65);
        public static final double depth = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(40.25);
        public static final double openingWidth = Units.inchesToMeters(50.34);
        public static final double openingHeight = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side
        public static final Translation3d openingTopLeft = new Translation3d(LinesVertical.hubCenter, fieldWidth,
                openingHeight);
        public static final Translation3d openingTopRight = new Translation3d(LinesVertical.hubCenter,
                fieldWidth - openingWidth, openingHeight);

        // Relevant reference points on opposing side
        public static final Translation3d oppOpeningTopLeft = new Translation3d(LinesVertical.oppHubCenter, fieldWidth,
                openingHeight);
        public static final Translation3d oppOpeningTopRight = new Translation3d(LinesVertical.oppHubCenter,
                fieldWidth - openingWidth, openingHeight);
    }

    public static class RightTrench {

        // Dimensions
        public static final double width = Units.inchesToMeters(65.65);
        public static final double depth = Units.inchesToMeters(47.0);
        public static final double height = Units.inchesToMeters(40.25);
        public static final double openingWidth = Units.inchesToMeters(50.34);
        public static final double openingHeight = Units.inchesToMeters(22.25);

        // Relevant reference points on alliance side
        public static final Translation3d openingTopLeft = new Translation3d(LinesVertical.hubCenter, openingWidth,
                openingHeight);
        public static final Translation3d openingTopRight = new Translation3d(LinesVertical.hubCenter, 0,
                openingHeight);

        // Relevant reference points on opposing side
        public static final Translation3d oppOpeningTopLeft = new Translation3d(LinesVertical.oppHubCenter,
                openingWidth, openingHeight);
        public static final Translation3d oppOpeningTopRight = new Translation3d(LinesVertical.oppHubCenter, 0,
                openingHeight);
    }

    /** Tower related constants */
    public static class Tower {
        // Dimensions
        public static final double width = Units.inchesToMeters(49.25);
        public static final double depth = Units.inchesToMeters(45.0);
        public static final double height = Units.inchesToMeters(78.25);
        public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
        public static final double frontFaceX = Units.inchesToMeters(43.51);

        public static final double uprightHeight = Units.inchesToMeters(72.1);

        // Rung heights from the floor
        public static final double lowRungHeight = Units.inchesToMeters(27.0);
        public static final double midRungHeight = Units.inchesToMeters(45.0);
        public static final double highRungHeight = Units.inchesToMeters(63.0);

        // Relevant reference points on alliance side
        public static final Translation2d centerPoint = new Translation2d(
                frontFaceX, layout.getConstLayout().getTagPose(31).get().getY());
        public static final Translation2d leftUpright = new Translation2d(
                frontFaceX,
                (layout.getConstLayout().getTagPose(31).get().getY())
                        + innerOpeningWidth / 2
                        + Units.inchesToMeters(0.75));
        public static final Translation2d rightUpright = new Translation2d(
                frontFaceX,
                (layout.getConstLayout().getTagPose(31).get().getY())
                        - innerOpeningWidth / 2
                        - Units.inchesToMeters(0.75));

        public static final Translation2d rightBackCorner = new Translation2d(
                0,
                rightUpright.getY());

        // Relevant reference points on opposing side
        public static final Translation2d oppCenterPoint = new Translation2d(
                fieldLength - frontFaceX,
                layout.getConstLayout().getTagPose(15).get().getY());
        public static final Translation2d oppLeftUpright = new Translation2d(
                fieldLength - frontFaceX,
                (layout.getConstLayout().getTagPose(15).get().getY())
                        + innerOpeningWidth / 2
                        + Units.inchesToMeters(0.75));
        public static final Translation2d oppRightUpright = new Translation2d(
                fieldLength - frontFaceX,
                (layout.getConstLayout().getTagPose(15).get().getY())
                        - innerOpeningWidth / 2
                        - Units.inchesToMeters(0.75));
    }

    public static class Depot {
        // Dimensions
        public static final double width = Units.inchesToMeters(42.0);
        public static final double depth = Units.inchesToMeters(27.0);
        public static final double height = Units.inchesToMeters(1.125);
        public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

        // Relevant reference points on alliance side
        public static final Translation3d depotCenter = new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY,
                height);
        public static final Translation3d leftCorner = new Translation3d(depth,
                (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
        public static final Translation3d rightCorner = new Translation3d(depth,
                (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
    }

    public static class Outpost {
        // Dimensions
        public static final double width = Units.inchesToMeters(31.8);
        public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
        public static final double height = Units.inchesToMeters(7.0);

        // Relevant reference points on alliance side
        public static final Translation2d centerPoint = new Translation2d(0,
                layout.getConstLayout().getTagPose(29).get().getY());
    }

    // layoutName is the layout used for vision without the json extension
    public enum AprilTagLayout {
        OFFICIAL_WELDED("2026-rebuilt-welded", false),
        OFFICIAL_ANDYMARK("2026-rebuilt-andymark", true),
        SHOP("field_calibration_shop", false); // i think it's based on welded

        private final String layoutName;
        private final boolean isAndymark;
        private volatile AprilTagFieldLayout constLayout;
        private volatile AprilTagFieldLayout visionLayout;

        AprilTagLayout(String layoutName, boolean isAndymark) {
            this.layoutName = layoutName;
            this.isAndymark = isAndymark;
        }

        /*
         * get the field layout for FieldConstants
         */
        public AprilTagFieldLayout getConstLayout() {
            if (constLayout == null) {
                synchronized (this) {
                    if (constLayout == null) {
                        constLayout = AprilTagFieldLayout.loadField(
                            isAndymark ?
                            AprilTagFields.k2026RebuiltAndymark :
                            AprilTagFields.k2026RebuiltWelded);
                    }
                }
            }
            return constLayout;
        }

        /*
         * get the field layout for vision processing
         */
        public AprilTagFieldLayout getVisionLayout() {
            if (visionLayout == null) {
                synchronized (this) {
                    if (visionLayout == null) {
                        try {
                            visionLayout = new AprilTagFieldLayout(
                                Path.of(
                                    Filesystem.getDeployDirectory().getPath(),
                                    layoutName + ".json"));
                            System.out.println("read apriltag file :D");
                        } catch (IOException e) {
                            e.printStackTrace();
                            System.out.println("COULD NOT READ APRILTAG FIELD FILE!!!!!!!!!!!!!!!!!!!!!!!!!");
                            visionLayout = getConstLayout();
                        }
                    }
                }
            }
            return visionLayout;
        }
    }
}