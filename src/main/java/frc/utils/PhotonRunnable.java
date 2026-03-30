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
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import frc.utils.PoseUtils.Heading;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

/** Runnable that gets AprilTag data from PhotonVision. */
public class PhotonRunnable implements Runnable {

    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera photonCamera;
    private AprilTagFieldLayout layout;
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
        layout = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();
        // Keep layout origin fixed to blue; alliance flipping is handled in PoseEstimatorSubsystem.
        // try {
        //     layout = new AprilTagFieldLayout(Filesystem.getDeployDirectory().getAbsolutePath() + "/field_calibration.json");
        //     System.out.println("COULD READ APRIL TAG FIELD FILE WOOOOOOOOOOOOO");
        // } catch (IOException e) {
        //     e.printStackTrace();
        //     System.out.println("COULD NOT READ APRIL TAG FIELD FILE!!!!!!!!!!!!!!!!!!!!!!!!!");
        //     layout = FieldConstants.AprilTagLayoutType.OFFICIAL.getLayout();
        // }
        layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);

        this.photonPoseEstimator = new PhotonPoseEstimator(layout, cameraToRobot);
    }

    @Override
    public void run() {
        if (photonPoseEstimator == null || photonCamera == null) {
            return;
        }

        boolean connected = photonCamera.isConnected();
        Logger.recordOutput("VisionCalibration/" + cameraName + "/Connected", connected);
        if (!connected) {
            photonResults = null;
            atomicTargetYaw.set(null);
            atomicEstimatedRobotPose.set(null);
            return;
        }

        var results = photonCamera.getAllUnreadResults();
        //Logger.recordOutput("VisionCalibration/" + cameraName + "/UnreadResultCount", results.size());
        EstimatedRobotPose newestPose = null;

        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) {
                continue;
            }

            var targets = result.getTargets();
            long[] tagIds = new long[targets.size()];
            Pose3d[] cameraToTags = new Pose3d[targets.size()];
            double[] ambiguities = new double[targets.size()];
            boolean[] tagInLayout = new boolean[targets.size()];

            for (int i = 0; i < targets.size(); i++) {
                var target = targets.get(i);
                tagIds[i] = target.getFiducialId();
                var transform = target.getBestCameraToTarget();
                cameraToTags[i] = new Pose3d(transform.getTranslation(), transform.getRotation());
                ambiguities[i] = target.getPoseAmbiguity();
                tagInLayout[i] = layout.getTagPose((int) tagIds[i]).isPresent();
            }

            //Logger.recordOutput(
            //        "VisionCalibration/" + cameraName + "/TimestampSec",
            //        result.getTimestampSeconds());
            //Logger.recordOutput("VisionCalibration/" + cameraName + "/TagIds", tagIds);
            //Logger.recordOutput("VisionCalibration/" + cameraName + "/TagInLayout", tagInLayout);
            //Logger.recordOutput("VisionCalibration/" + cameraName + "/CameraToTag", cameraToTags);
            //Logger.recordOutput("VisionCalibration/" + cameraName + "/Ambiguity", ambiguities);

            photonResults = result;
            atomicTargetYaw.set(result.getBestTarget().yaw);

            var photonPose = photonPoseEstimator
                    .estimateCoprocMultiTagPose(result)
                    .or(() -> photonPoseEstimator.estimateLowestAmbiguityPose(result));

            //if (photonPose.isEmpty()) {
                //int bestId = result.getBestTarget().getFiducialId();
                //boolean bestIdInLayout = layout.getTagPose(bestId).isPresent();
                //Logger.recordOutput("VisionCalibration/" + cameraName + "/PoseEstimateEmpty", true);
                //Logger.recordOutput("VisionCalibration/" + cameraName + "/BestTargetId", bestId);
                //Logger.recordOutput("VisionCalibration/" + cameraName + "/BestTargetInLayout", bestIdInLayout);
                //continue;
            //}
            //Logger.recordOutput("VisionCalibration/" + cameraName + "/PoseEstimateEmpty", false);

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
