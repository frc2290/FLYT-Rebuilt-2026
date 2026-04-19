package frc.utils;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

class VisionSimTest {
    private static final double FIELD_LENGTH_METERS = 16.541;
    private static final double FIELD_WIDTH_METERS = 8.069;
    private static int simInstanceCounter = 0;

    private static final class FourCameraRig {
        private final VisionSystemSim visionSim;
        private final PhotonCamera frontCam;
        private final PhotonCamera backCam;
        private final PhotonCamera leftCam;
        private final PhotonCamera rightCam;

        private FourCameraRig(
                VisionSystemSim visionSim,
                PhotonCamera frontCam,
                PhotonCamera backCam,
                PhotonCamera leftCam,
                PhotonCamera rightCam) {
            this.visionSim = visionSim;
            this.frontCam = frontCam;
            this.backCam = backCam;
            this.leftCam = leftCam;
            this.rightCam = rightCam;
        }
    }

    @BeforeEach
    void setUp() {
        HAL.initialize(500, 0);
        SimHooks.pauseTiming();
        SimHooks.restartTiming();
    }

    @AfterEach
    void tearDown() {
        SimHooks.resumeTiming();
    }

    @Test
    void forwardCameraSeesTagDirectlyInFront() {
        Pose2d robotPose = new Pose2d(8.0, 5.0, Rotation2d.fromDegrees(0.0));
        PhotonCamera camera = new PhotonCamera("dummy-forward-visible");
        VisionSystemSim visionSim = createVisionSim(
                "TestWorld-" + simInstanceCounter++,
                camera,
                robotPose,
                new Transform3d(new Translation3d(3.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, Math.PI)));

        stepAndUpdate(visionSim, robotPose, 10);
        boolean sawTarget = hasAnyTarget(camera);
        assertTrue(sawTarget, "The forward camera failed to see a tag directly in front of it.");
    }

    @Test
    void forwardCameraDoesNotSeeTagBehindIt() {
        Pose2d robotPose = new Pose2d(8.0, 5.0, Rotation2d.fromDegrees(0.0));
        PhotonCamera camera = new PhotonCamera("dummy-forward-behind");
        VisionSystemSim visionSim = createVisionSim(
                "TestWorld-" + simInstanceCounter++,
                camera,
                robotPose,
                new Transform3d(new Translation3d(-3.0, 0.0, 0.0), new Rotation3d()));

        stepAndUpdate(visionSim, robotPose, 10);
        assertFalse(hasAnyTarget(camera), "The forward camera should not detect a tag behind its optical axis.");
    }

    @Test
    void forwardCameraCenteredTagProducesNearZeroYaw() {
        Pose2d robotPose = new Pose2d(8.0, 5.0, Rotation2d.fromDegrees(0.0));
        PhotonCamera camera = new PhotonCamera("dummy-forward-yaw");
        VisionSystemSim visionSim = createVisionSim(
                "TestWorld-" + simInstanceCounter++,
                camera,
                robotPose,
                new Transform3d(new Translation3d(3.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, Math.PI)));

        stepAndUpdate(visionSim, robotPose, 10);

        double minAbsYawDeg = camera.getAllUnreadResults().stream()
                .filter(result -> result.hasTargets())
                .mapToDouble(result -> Math.abs(result.getBestTarget().yaw))
                .min()
                .orElse(Double.POSITIVE_INFINITY);
        assertTrue(minAbsYawDeg < 1.0, "Centered target should have near-zero yaw.");
    }

    @Test
    void cameraLatencyDelaysNewTargetFrames() {
        Pose2d hiddenPose = new Pose2d(8.0, 5.0, Rotation2d.fromDegrees(180.0));
        Pose2d visiblePose = new Pose2d(8.0, 5.0, Rotation2d.fromDegrees(0.0));
        Transform3d visibleTagFromVisiblePose =
                new Transform3d(new Translation3d(3.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, Math.PI));

        PhotonCamera camera = new PhotonCamera("dummy-latency-delay");
        VisionSystemSim visionSim = createVisionSim(
                "TestWorld-" + simInstanceCounter++, camera, visiblePose, visibleTagFromVisiblePose, 120.0);

        stepAndUpdate(visionSim, hiddenPose, 10);
        assertFalse(hasAnyTarget(camera), "Tag should not be visible while the robot is turned away.");
        camera.getAllUnreadResults();

        stepAndUpdate(visionSim, visiblePose, 3);
        assertFalse(hasAnyTarget(camera), "Latency should prevent new target frames from arriving immediately.");

        stepAndUpdate(visionSim, visiblePose, 8);
        assertTrue(hasAnyTarget(camera), "After latency elapses, target frames should arrive.");
    }

    @Test
    void forwardCameraVisibilityAtZeroDegrees() {
        FourCameraRig rig = createFourCameraRig();

        stepAndUpdate(rig.visionSim, new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(0.0)), 5);
        assertTrue(
                hasAnyTarget(rig.frontCam),
                "Forward camera failed to see the tag with robot at (3,5) facing 0 degrees.");
    }

    @Test
    void leftCameraVisibilityAtTwoSeventyDegrees() {
        FourCameraRig rig = createFourCameraRig();

        stepAndUpdate(rig.visionSim, new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(270.0)), 5);
        assertTrue(
                hasAnyTarget(rig.leftCam),
                "Left camera failed to see the tag with robot at (3,5) facing 270 degrees.");
    }

    @Test
    void backwardCameraVisibilityAtOneEightyDegrees() {
        FourCameraRig rig = createFourCameraRig();

        stepAndUpdate(rig.visionSim, new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(180.0)), 5);
        assertTrue(
                hasAnyTarget(rig.backCam),
                "Backward camera failed to see the tag with robot at (3,5) facing 180 degrees.");
    }

    @Test
    void rightCameraVisibilityAtNinetyDegrees() {
        FourCameraRig rig = createFourCameraRig();

        stepAndUpdate(rig.visionSim, new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(90.0)), 5);
        assertTrue(
                hasAnyTarget(rig.rightCam),
                "Right camera failed to see the tag with robot at (3,5) facing 90 degrees.");
    }

    @Test
    void simulatedNoiseSpikesAmbiguityAboveThreshold() {
        VisionSystemSim noisySim = new VisionSystemSim("NoisyWorld-" + simInstanceCounter++);
        AprilTag fakeTag = new AprilTag(
                1,
                new Pose3d(
                        5.0,
                        5.0,
                        VisionConstants.TARGET_HEIGHT_METERS,
                        new Rotation3d(0.0, 0.0, Math.PI)));
        noisySim.addAprilTags(new AprilTagFieldLayout(List.of(fakeTag), FIELD_LENGTH_METERS, FIELD_WIDTH_METERS));

        PhotonCamera noisyCam = new PhotonCamera("noisy-front-" + simInstanceCounter++);
        noisySim.addCamera(createSimCam(noisyCam, 0.25, 0.08, 0.0), VisionConstants.kForwardCamTransform);

        Pose2d[] probePoses = new Pose2d[] {
                new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(2.5, 5.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(2.0, 5.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(1.5, 5.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(1.0, 5.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(0.5, 5.0, Rotation2d.fromDegrees(0.0)),
                new Pose2d(3.0, 4.5, Rotation2d.fromDegrees(0.0)),
                new Pose2d(3.0, 5.5, Rotation2d.fromDegrees(0.0)),
                new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(15.0)),
                new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(-15.0)),
                new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(30.0)),
                new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(-30.0))
        };
        int framesRejectedByAmbiguity = 0;
        int totalValidFrames = 0;
        double maxAmbiguity = Double.NEGATIVE_INFINITY;

        for (Pose2d robotPose : probePoses) {
            for (int i = 0; i < 30; i++) {
                SimHooks.stepTiming(0.02);
                noisySim.update(robotPose);

                for (var result : noisyCam.getAllUnreadResults()) {
                    if (!result.hasTargets()) {
                        continue;
                    }
                    totalValidFrames++;
                    double ambiguity = result.getBestTarget().getPoseAmbiguity();
                    if (ambiguity > maxAmbiguity) {
                        maxAmbiguity = ambiguity;
                    }
                    if (ambiguity > VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD) {
                        framesRejectedByAmbiguity++;
                    }
                }
            }
        }

        assertTrue(totalValidFrames > 0, "Camera could not see the tag at all.");
        assertTrue(
                framesRejectedByAmbiguity > 0,
                "Expected noisy single-tag frames above ambiguity threshold, but saw 0 rejections out of "
                        + totalValidFrames + " valid frames. Max ambiguity observed: " + maxAmbiguity);
    }

    @Test
    void distanceFilterTriggersOnFarTags() {
        VisionSystemSim sim = new VisionSystemSim("DistanceWorld-" + simInstanceCounter++);
        AprilTag fakeTag = new AprilTag(
                1,
                new Pose3d(
                        5.0,
                        5.0,
                        VisionConstants.TARGET_HEIGHT_METERS,
                        new Rotation3d(0.0, 0.0, Math.PI)));
        sim.addAprilTags(new AprilTagFieldLayout(List.of(fakeTag), FIELD_LENGTH_METERS, FIELD_WIDTH_METERS));

        PhotonCamera frontCam = new PhotonCamera("distance-front-" + simInstanceCounter++);
        sim.addCamera(createZeroLatencySimCam(frontCam), VisionConstants.kForwardCamTransform);

        Pose2d farPose = new Pose2d(0.5, 5.0, Rotation2d.fromDegrees(0.0));
        stepAndUpdate(sim, farPose, 10);

        boolean sawTarget = false;
        double maxDistanceMeters = 0.0;
        for (var result : frontCam.getAllUnreadResults()) {
            if (!result.hasTargets()) {
                continue;
            }
            sawTarget = true;
            double distMeters = result.getBestTarget().getBestCameraToTarget().getTranslation().getNorm();
            if (distMeters > maxDistanceMeters) {
                maxDistanceMeters = distMeters;
            }
        }

        assertTrue(sawTarget, "Camera should still see the tag at far distance.");
        assertTrue(
                maxDistanceMeters > 4.0,
                "Expected best-camera-to-target distance > 4.0m, got " + maxDistanceMeters + "m.");
    }

    @Test
    void multiTagDetectionBypassesSingleTagFilters() {
        VisionSystemSim multiTagSim = new VisionSystemSim("MultiTagWorld-" + simInstanceCounter++);
        AprilTag tag1 = new AprilTag(
                1,
                new Pose3d(
                        5.0,
                        4.5,
                        VisionConstants.TARGET_HEIGHT_METERS,
                        new Rotation3d(0.0, 0.0, Math.PI)));
        AprilTag tag2 = new AprilTag(
                2,
                new Pose3d(
                        5.0,
                        5.5,
                        VisionConstants.TARGET_HEIGHT_METERS,
                        new Rotation3d(0.0, 0.0, Math.PI)));
        multiTagSim.addAprilTags(
                new AprilTagFieldLayout(List.of(tag1, tag2), FIELD_LENGTH_METERS, FIELD_WIDTH_METERS));

        PhotonCamera multiCam = new PhotonCamera("multi-front-" + simInstanceCounter++);
        multiTagSim.addCamera(createZeroLatencySimCam(multiCam), VisionConstants.kForwardCamTransform);

        stepAndUpdate(multiTagSim, new Pose2d(2.5, 5.0, Rotation2d.fromDegrees(0.0)), 10);

        int maxTargetCount = 0;
        for (var result : multiCam.getAllUnreadResults()) {
            if (result.getTargets().size() > maxTargetCount) {
                maxTargetCount = result.getTargets().size();
            }
        }

        assertTrue(
                maxTargetCount > 1,
                "Expected multi-tag detection (>1 target) to bypass single-tag-only filter path.");
    }

    @Test
    void extraAllianceFlipCanMoveRobotOutOfTagView() {
        VisionSystemSim sim = new VisionSystemSim("FlipMismatchWorld-" + simInstanceCounter++);
        AprilTag fakeTag = new AprilTag(
                1,
                new Pose3d(
                        5.0,
                        5.0,
                        VisionConstants.TARGET_HEIGHT_METERS,
                        new Rotation3d(0.0, 0.0, Math.PI)));
        sim.addAprilTags(new AprilTagFieldLayout(List.of(fakeTag), FIELD_LENGTH_METERS, FIELD_WIDTH_METERS));

        PhotonCamera cam = new PhotonCamera("flip-mismatch-front-" + simInstanceCounter++);
        sim.addCamera(createZeroLatencySimCam(cam), VisionConstants.kForwardCamTransform);

        Pose2d odometryPoseBlue = new Pose2d(3.0, 5.0, Rotation2d.fromDegrees(0.0));
        Pose2d incorrectlyFlippedPose = odometryPoseBlue.relativeTo(VisionConstants.FLIPPING_POSE);

        stepAndUpdate(sim, odometryPoseBlue, 10);
        boolean seesTagAtOdometryPose = hasAnyTarget(cam);

        cam.getAllUnreadResults();
        stepAndUpdate(sim, incorrectlyFlippedPose, 10);
        boolean seesTagAtIncorrectlyFlippedPose = hasAnyTarget(cam);

        assertTrue(seesTagAtOdometryPose, "Expected visible tag at the true odometry pose.");
        assertFalse(
                seesTagAtIncorrectlyFlippedPose,
                "Applying an extra alliance flip moved the robot to a pose with no visible target.");
    }

    private VisionSystemSim createVisionSim(
            String simName, PhotonCamera camera, Pose2d robotPose, Transform3d cameraToTag) {
        return createVisionSim(simName, camera, robotPose, cameraToTag, 0.0);
    }

    private VisionSystemSim createVisionSim(
            String simName,
            PhotonCamera camera,
            Pose2d robotPose,
            Transform3d cameraToTag,
            double avgLatencyMs) {
        VisionSystemSim visionSim = new VisionSystemSim(simName);

        Pose3d cameraPose = new Pose3d(robotPose).transformBy(VisionConstants.kForwardCamTransform);
        Pose3d fakeTagPose = cameraPose.transformBy(cameraToTag);
        AprilTag fakeTag = new AprilTag(1, fakeTagPose);
        AprilTagFieldLayout layout =
                new AprilTagFieldLayout(List.of(fakeTag), FIELD_LENGTH_METERS, FIELD_WIDTH_METERS);
        visionSim.addAprilTags(layout);

        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(1280, 800, Rotation2d.fromDegrees(75));
        props.setCalibError(0.0, 0.0);
        props.setFPS(40);
        props.setAvgLatencyMs(avgLatencyMs);
        props.setLatencyStdDevMs(0.0);

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, props);
        cameraSim.enableRawStream(false);
        cameraSim.enableProcessedStream(false);
        visionSim.addCamera(cameraSim, VisionConstants.kForwardCamTransform);
        return visionSim;
    }

    private FourCameraRig createFourCameraRig() {
        VisionSystemSim visionSim = new VisionSystemSim("TestWorld-" + simInstanceCounter++);
        AprilTag fakeTag = new AprilTag(
                1,
                new Pose3d(
                        5.0,
                        5.0,
                        VisionConstants.TARGET_HEIGHT_METERS,
                        new Rotation3d(0.0, 0.0, Math.PI)));
        AprilTagFieldLayout layout =
                new AprilTagFieldLayout(List.of(fakeTag), FIELD_LENGTH_METERS, FIELD_WIDTH_METERS);
        visionSim.addAprilTags(layout);

        PhotonCamera frontCam = new PhotonCamera("front-" + simInstanceCounter++);
        PhotonCamera backCam = new PhotonCamera("back-" + simInstanceCounter++);
        PhotonCamera leftCam = new PhotonCamera("left-" + simInstanceCounter++);
        PhotonCamera rightCam = new PhotonCamera("right-" + simInstanceCounter++);

        visionSim.addCamera(createZeroLatencySimCam(frontCam), VisionConstants.kForwardCamTransform);
        visionSim.addCamera(createZeroLatencySimCam(backCam), VisionConstants.kBackwardCamTransform);
        visionSim.addCamera(createZeroLatencySimCam(leftCam), VisionConstants.kLeftCamTransform);
        visionSim.addCamera(createZeroLatencySimCam(rightCam), VisionConstants.kRightCamTransform);

        return new FourCameraRig(visionSim, frontCam, backCam, leftCam, rightCam);
    }

    private static PhotonCameraSim createZeroLatencySimCam(PhotonCamera camera) {
        return createSimCam(camera, 0.0, 0.0, 0.0);
    }

    private static PhotonCameraSim createSimCam(
            PhotonCamera camera, double calibErrPx, double calibErrDeg, double avgLatencyMs) {
        SimCameraProperties props = new SimCameraProperties();
        props.setCalibration(1280, 800, Rotation2d.fromDegrees(75));
        props.setCalibError(calibErrPx, calibErrDeg);
        props.setFPS(40);
        props.setAvgLatencyMs(avgLatencyMs);
        props.setLatencyStdDevMs(0.0);

        PhotonCameraSim cameraSim = new PhotonCameraSim(camera, props);
        cameraSim.enableRawStream(false);
        cameraSim.enableProcessedStream(false);
        return cameraSim;
    }

    private static void stepAndUpdate(VisionSystemSim visionSim, Pose2d robotPose, int cycles) {
        for (int i = 0; i < cycles; i++) {
            SimHooks.stepTiming(0.02);
            visionSim.update(robotPose);
        }
    }

    private static boolean hasAnyTarget(PhotonCamera camera) {
        return camera.getAllUnreadResults().stream().anyMatch(result -> result.hasTargets());
    }
}
