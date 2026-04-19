package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.VisionConstants;
import org.junit.jupiter.api.Test;

class PoseUtilsTest {

    private Pose2d flipAlliance(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(VisionConstants.FLIPPING_POSE);
    }

    @Test
    void allianceFlipMirrorsAcrossCenterline() {
        Pose2d redPose = new Pose2d(1.0, 2.0, Rotation2d.fromDegrees(180.0));

        Pose2d absoluteBluePose = flipAlliance(redPose);

        assertEquals(15.541, absoluteBluePose.getX(), 0.001);
        assertEquals(6.069, absoluteBluePose.getY(), 0.001);
        assertEquals(0.0, absoluteBluePose.getRotation().getDegrees(), 0.001);
    }

    @Test
    void allianceFlipTwiceReturnsOriginalPose() {
        Pose2d[] samples = {
                new Pose2d(0.5, 1.2, Rotation2d.fromDegrees(30.0)),
                new Pose2d(4.0, 6.1, Rotation2d.fromDegrees(-100.0)),
                new Pose2d(15.0, 7.8, Rotation2d.fromDegrees(179.0))
        };

        for (Pose2d sample : samples) {
            Pose2d roundTrip = flipAlliance(flipAlliance(sample));
            assertEquals(sample.getX(), roundTrip.getX(), 1e-9);
            assertEquals(sample.getY(), roundTrip.getY(), 1e-9);
            double rotationErrorDeg = Math.abs(sample.getRotation().minus(roundTrip.getRotation()).getDegrees());
            assertTrue(rotationErrorDeg < 1e-9, "Double flip must preserve heading.");
        }
    }

    @Test
    void allianceFlipUsesFieldDimensionsForTranslationMirror() {
        Pose2d pose = new Pose2d(2.25, 3.5, Rotation2d.fromDegrees(95.0));

        Pose2d flipped = flipAlliance(pose);

        assertEquals(VisionConstants.FIELD_LENGTH_METERS - pose.getX(), flipped.getX(), 1e-9);
        assertEquals(VisionConstants.FIELD_WIDTH_METERS - pose.getY(), flipped.getY(), 1e-9);
    }
}
