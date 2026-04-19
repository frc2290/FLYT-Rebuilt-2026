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
import edu.wpi.first.math.geometry.Transform3d;
import java.util.concurrent.atomic.AtomicReference;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
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

    /** Latest target yaw published from the PhotonVision thread. */
    private final AtomicReference<Double> atomicTargetYaw = new AtomicReference<Double>();

    /** Cached pipeline result from the most recent update. */
    private volatile PhotonPipelineResult photonResults;

    public PhotonRunnable(String cam_name, Transform3d cameraToRobot, AprilTagFieldLayout sharedLayout) {
        cameraName = cam_name;
        this.photonCamera = new PhotonCamera(cameraName);
        this.layout = sharedLayout;
        // The pose estimator outputs absolute (blue-origin) field poses.
        this.photonPoseEstimator = new PhotonPoseEstimator(layout, cameraToRobot);
    }

    @Override
    public void run() {
        if (photonPoseEstimator == null || photonCamera == null) {
            return;
        }

        boolean connected = photonCamera.isConnected();
        Logger.recordOutput("Vision/" + cameraName + "/Connected", connected);
        if (!connected) {
            photonResults = null;
            atomicTargetYaw.set(null);
            atomicEstimatedRobotPose.set(null);
            return;
        }

        var results = photonCamera.getAllUnreadResults();
        EstimatedRobotPose newestPose = null;

        for (PhotonPipelineResult result : results) {
            // Keep latest pipeline frame even when it has no targets, so downstream logs can clear correctly.
            photonResults = result;

            if (!result.hasTargets()) {
                continue;
            }

            atomicTargetYaw.set(result.getBestTarget().yaw);

            var photonPose = photonPoseEstimator
                    .estimateCoprocMultiTagPose(result)
                    .or(() -> photonPoseEstimator.estimateLowestAmbiguityPose(result));

            // Prevent unhandled NoSuchElementExceptions from silently killing the Notifier thread.
            if (photonPose.isEmpty()) {
                continue;
            }

            var est = photonPose.get();
            // Log the raw 3D output before filtering so geometry issues are visible.
            Logger.recordOutput("Vision/RawThreadPoses/" + cameraName, est.estimatedPose);

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

    public PhotonPipelineResult grabLatestTag() {
        return photonResults;
    }

    public PhotonCamera getCamera() {
        return photonCamera;
    }
}
