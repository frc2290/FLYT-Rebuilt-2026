// Copyright (c) 2025 FRC 2290
// http://https://github.com/frc2290
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as
// published by the Free Software Foundation, either version 3 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
package frc.utils;

import static frc.robot.Constants.VisionConstants.kVisionMaxPoseZMeters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.utils.PoseUtils.Heading;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

/** Runnable that gets AprilTag data from PhotonVision. */
public class PhotonRunnable implements Runnable {

    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera photonCamera;
    private final AprilTagFieldLayout layout;
    private final String cameraName;

    /** Latest pose estimate published from the PhotonVision thread. */
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose =
            new AtomicReference<EstimatedRobotPose>();

    /** java is evil */
    private final AtomicReference<Double> atomicTargetYaw = new AtomicReference<Double>();

    /** Cached pipeline result from the last successful update. */
    private volatile PhotonPipelineResult photonResults;

    public PhotonRunnable(
            String cam_name, Transform3d cameraToRobot, Supplier<Heading> headingSupplier) {
        cameraName = cam_name;
        this.photonCamera = new PhotonCamera(cameraName);
        // Keep layout origin fixed to blue; alliance flipping is handled in PoseEstimatorSubsystem.
        layout = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        this.photonPoseEstimator = new PhotonPoseEstimator(
                layout,
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraToRobot);
    }

    @Override
    public void run() {
        if (photonPoseEstimator == null || photonCamera == null) {
            return;
        }

        var results = photonCamera.getAllUnreadResults();
        EstimatedRobotPose newestPose = null;

        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) {
                continue;
            }

            photonResults = result;
            atomicTargetYaw.set(result.getBestTarget().yaw);

            var photonPose = photonPoseEstimator
                    .estimateCoprocMultiTagPose(result)
                    .or(() -> photonPoseEstimator.estimateLowestAmbiguityPose(result));

            if (photonPose.isEmpty()) {
                continue;
            }

            var est = photonPose.get();
            if (Math.abs(est.estimatedPose.getZ()) > kVisionMaxPoseZMeters) {
                continue;
            }

            if (newestPose == null || est.timestampSeconds > newestPose.timestampSeconds) {
                newestPose = est;
            }
        }

        if (newestPose != null) {
            EstimatedRobotPose current = atomicEstimatedRobotPose.get();
            if (current == null || newestPose.timestampSeconds > current.timestampSeconds) {
                atomicEstimatedRobotPose.set(newestPose);
            }
        }
    }

    public EstimatedRobotPose grabLatestUpdate() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }

    /**
     * Gets the latest robot pose. Calling this will only return the pose once. If
     * it returns a
     * non-null value, it is a new estimate that hasn't been returned before. This
     * pose will always be
     * for the BLUE alliance. It must be flipped if the current alliance is RED.
     *
     * @return latest estimated pose
     */
    public EstimatedRobotPose grabLatestEstimatedPose() {
        return grabLatestUpdate();
    }

    public Double getTargetYaw() {
        return atomicTargetYaw.getAndSet(null);
    }

    public Pose2d grabLatestResult() {
        if (photonResults == null || !photonResults.hasTargets()) {
            return null;
        }

        return layout.getTagPose(photonResults.getBestTarget().getFiducialId())
                .map(pose3d -> pose3d.toPose2d())
                .orElse(null);
    }

    public PhotonPipelineResult grabLatestTag() {
        return photonResults;
    }

    public String getCameraName() {
        return cameraName;
    }
}
