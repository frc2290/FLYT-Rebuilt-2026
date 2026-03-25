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

import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide;
import static edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
// import edu.wpi.first.math.kinematics.SwerveModulePosition;
// import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;
// import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
// import frc.utils.FLYTLib.FLYTDashboard.FlytLogger;
import frc.utils.PoseUtils.Heading;
// import java.util.function.Supplier;
import org.photonvision.EstimatedRobotPose;

import org.littletonrobotics.junction.AutoLogOutput;
// import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.Logger;

/** Pose estimator that uses odometry and AprilTags with PhotonVision. */
public class PoseEstimatorSubsystem extends SubsystemBase {

    private static final class VisionMeasurement {
        private final String cameraName;
        private final Pose2d pose;
        private final double timestampSeconds;
        private final Matrix<N3, N1> stdDevs;
        private final double stdDevScore;

        private VisionMeasurement(
                String cameraName,
                Pose2d pose,
                double timestampSeconds,
                Matrix<N3, N1> stdDevs,
                double stdDevScore) {
            this.cameraName = cameraName;
            this.pose = pose;
            this.timestampSeconds = timestampSeconds;
            this.stdDevs = stdDevs;
            this.stdDevScore = stdDevScore;
        }
    }

    // Kalman Filter configuration. These can be "tuned-to-taste" based on how much
    // you trust your various sensors. Smaller numbers will cause the filter to
    // "trust" the estimate from that particular component more than the others.
    // This in turn means the particular component will have a stronger influence
    // on the final pose estimate.

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in
     * meters and radians,
     * then meters.
     */
    // private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.01); // VecBuilder.fill(0.1, 0.1, 0.1).
    // /** Supplier for the current gyro heading. */
    // private final Supplier<Rotation2d> rotationSupplier;

    // /** Supplier for the swerve module positions used in odometry updates. */
    // private final Supplier<SwerveModulePosition[]> modulePositionSupplier;

    // /** Supplier for module states so commands can query current wheel speeds. */
    // private final Supplier<SwerveModuleState[]> moduleStateSupplier;

    //private final SwerveDrivePoseEstimator poseEstimator;

    /** Field visualization that displays the robot pose in AdvantageScope. */
    // private final Field2d field2d = new Field2d();

    // private final FieldObject2d target2d = field2d.getObject("Target");

    /** target yaw for pointing */
    private double targetYaw = 0;

    /** PhotonVision pipelines for each camera in the four-camera rig. */
    private final PhotonRunnable frontPhotonRunnable;
    private final PhotonRunnable backPhotonRunnable;
    private final PhotonRunnable leftPhotonRunnable;
    private final PhotonRunnable rightPhotonRunnable;

    private final Notifier frontPhotonNotifier;
    private final Notifier backPhotonNotifier;
    private final Notifier leftPhotonNotifier;
    private final Notifier rightPhotonNotifier;

    private OriginPosition originPosition = kBlueAllianceWallRightSide;
    private boolean sawTag = false;

    private RobotConfig config;

    private Drive drive;

    /**
     * Pose that the drivetrain should aim toward (used by auto alignment commands).
     */
    private Pose2d targetPose = new Pose2d();

    /** Logger that streams pose data for tuning and match review. */
    // private FlytLogger poseDash = new FlytLogger("Pose");

    public PoseEstimatorSubsystem(Drive m_drive) {
        this.drive = m_drive;

        frontPhotonRunnable = new PhotonRunnable(
                VisionConstants.kForwardCamName, VisionConstants.kForwardCamTransform, () -> getHeading());
        backPhotonRunnable = new PhotonRunnable(
                VisionConstants.kBackwardCamName, VisionConstants.kForwardCamTransform2, () -> getHeading()); //backward camera
        leftPhotonRunnable = new PhotonRunnable(
                VisionConstants.kLeftCamName, VisionConstants.kLeftCamTransform, () -> getHeading());
        rightPhotonRunnable = new PhotonRunnable(
                VisionConstants.kRightCamName, VisionConstants.kRightCamTransform, () -> getHeading());

        frontPhotonNotifier = new Notifier(frontPhotonRunnable);
        backPhotonNotifier = new Notifier(backPhotonRunnable);
        leftPhotonNotifier = new Notifier(leftPhotonRunnable);
        rightPhotonNotifier = new Notifier(rightPhotonRunnable);

        // Start PhotonVision threads.
        frontPhotonNotifier.setName("PhotonRunnableForward");
        backPhotonNotifier.setName("PhotonRunnableBackward");
        leftPhotonNotifier.setName("PhotonRunnableLeft");
        rightPhotonNotifier.setName("PhotonRunnableRight");

        frontPhotonNotifier.startPeriodic(0.01);
        backPhotonNotifier.startPeriodic(0.01);
        leftPhotonNotifier.startPeriodic(0.01);
        rightPhotonNotifier.startPeriodic(0.01);

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    /** Updates the AprilTag origin based on the current alliance color. */
    public void setAlliance(Alliance alliance) {
        boolean allianceChanged = false;
        switch (alliance) {
            case Blue:
                allianceChanged = (originPosition == kRedAllianceWallRightSide);
                originPosition = kBlueAllianceWallRightSide;
                break;
            case Red:
                allianceChanged = (originPosition == kBlueAllianceWallRightSide);
                originPosition = kRedAllianceWallRightSide;
                break;
            default:
                // No valid alliance data. Nothing we can do about it.
        }

        if (allianceChanged && sawTag) {
            // The alliance changed, which changes the coordinate system.
            // Since a tag was seen, and the tags are all relative to the coordinate system,
            // the estimated pose needs to be transformed to the new coordinate system.
            var newPose = flipAlliance(getCurrentPose());
            drive.setPose(newPose);
        }
    }

    @Override
    public void periodic() {
        // Update pose estimator with drivetrain sensors.
        //poseEstimator.update(rotationSupplier.get(), modulePositionSupplier.get());

        EstimatedRobotPose frontUpdate = frontPhotonRunnable.grabLatestUpdate();
        EstimatedRobotPose backUpdate = backPhotonRunnable.grabLatestUpdate();
        EstimatedRobotPose leftUpdate = leftPhotonRunnable.grabLatestUpdate();
        EstimatedRobotPose rightUpdate = rightPhotonRunnable.grabLatestUpdate();

        VisionMeasurement bestMeasurement = null;
        bestMeasurement = chooseBetter(bestMeasurement, processCameraUpdate(frontUpdate, "Front"));
        bestMeasurement = chooseBetter(bestMeasurement, processCameraUpdate(backUpdate, "Back"));
        bestMeasurement = chooseBetter(bestMeasurement, processCameraUpdate(leftUpdate, "Left"));
        bestMeasurement = chooseBetter(bestMeasurement, processCameraUpdate(rightUpdate, "Right"));

        if (bestMeasurement != null) {
            drive.addVisionMeasurement(
                    bestMeasurement.pose,
                    bestMeasurement.timestampSeconds,
                    bestMeasurement.stdDevs);
            sawTag = true;
            Logger.recordOutput("Vision/SelectedCamera", bestMeasurement.cameraName);
            Logger.recordOutput("Vision/SelectedPose", bestMeasurement.pose);
            Logger.recordOutput("Vision/SelectedStdDevScore", bestMeasurement.stdDevScore);
        } else {
            Logger.recordOutput("Vision/SelectedCamera", "");
            Logger.recordOutput("Vision/SelectedPose", new Pose2d());
            Logger.recordOutput("Vision/SelectedStdDevScore", Double.NaN);
        }
    }

    /** Processes a single camera frame and returns a measurement candidate (or null). */
    private VisionMeasurement processCameraUpdate(EstimatedRobotPose update, String cameraName) {
        if (update == null || update.targetsUsed == null || update.targetsUsed.isEmpty()) {
            Logger.recordOutput("Vision/CameraPoses/" + cameraName, new Pose2d[] {});
            return null;
        }

        // Apply legacy hard cutoffs only for single-tag solves.
        if (update.targetsUsed.size() == 1) {
            var target = update.targetsUsed.get(0);
            double distMeters = target.getBestCameraToTarget().getTranslation().getNorm();
            double ambiguity = target.getPoseAmbiguity();
            if (distMeters > 3.0 || ambiguity > VisionConstants.APRILTAG_AMBIGUITY_THRESHOLD) {
                Logger.recordOutput("Vision/CameraPoses/" + cameraName, new Pose2d[] {});
                return null;
            }
        }

        Matrix<N3, N1> stdDevs = calculateStdDevs(update);
        double varX = Math.pow(stdDevs.get(0, 0), 2);
        if (varX >= VisionConstants.kVisionRejectVarianceThreshold) {
            Logger.recordOutput("Vision/CameraPoses/" + cameraName, new Pose2d[] {});
            return null;
        }

        Pose2d finalPose = update.estimatedPose.toPose2d();
        if (originPosition != kBlueAllianceWallRightSide) {
            finalPose = flipAlliance(finalPose);
        }

        Logger.recordOutput("Vision/CameraPoses/" + cameraName, new Pose2d[] {finalPose});

        double score = stdDevScore(stdDevs);
        return new VisionMeasurement(cameraName, finalPose, update.timestampSeconds, stdDevs, score);
    }

    private VisionMeasurement chooseBetter(VisionMeasurement currentBest, VisionMeasurement candidate) {
        if (candidate == null) {
            return currentBest;
        }
        if (currentBest == null) {
            return candidate;
        }
        return candidate.stdDevScore < currentBest.stdDevScore ? candidate : currentBest;
    }

    private static double stdDevScore(Matrix<N3, N1> stdDevs) {
        double x = stdDevs.get(0, 0);
        double y = stdDevs.get(1, 0);
        double theta = stdDevs.get(2, 0);
        return (x * x) + (y * y) + (theta * theta);
    }

    /** Calculates dynamic standard deviations using multiplicative scaling. */
    private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose est) {
        var targets = est.targetsUsed;
        int numTags = targets.size();

        if (numTags == 0) {
            return VecBuilder.fill(
                    VisionConstants.kVisionInvalidStdDev,
                    VisionConstants.kVisionInvalidStdDev,
                    VisionConstants.kVisionInvalidStdDev);
        }

        double avgDist = 0.0;
        for (var target : targets) {
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDist /= numTags;

        double xyStd = VisionConstants.kVisionXyStdDevCoefficient
                * Math.pow(avgDist, VisionConstants.kVisionStdDevDistanceExponent)
                / Math.pow(numTags, VisionConstants.kVisionStdDevTagCountExponent)
                * VisionConstants.kVisionCameraStdDevFactor;

        double rotStd = VisionConstants.kVisionThetaStdDevCoefficient
                * Math.pow(avgDist, VisionConstants.kVisionStdDevDistanceExponent)
                / Math.pow(numTags, VisionConstants.kVisionStdDevTagCountExponent)
                * VisionConstants.kVisionCameraStdDevFactor;

        xyStd = Math.max(xyStd, VisionConstants.kVisionStdDevMin);
        rotStd = Math.max(rotStd, VisionConstants.kVisionStdDevMin);

        return VecBuilder.fill(xyStd, xyStd, rotStd);
    }

    public double getAlignX(Translation2d target) {
        return target.getX() - drive.getPose().getX();
    }

    public double getAlignY(Translation2d target) {
        return target.getY() - drive.getPose().getY();
    }

    public double turnToTarget(Translation2d target) {
        double offsetX = target.getX() - drive.getPose().getX();
        double offsetY = target.getY() - drive.getPose().getY();
        // return (360 - Math.toDegrees(Math.atan2(offsetY, offsetX)) % 360);
        return (Math.toDegrees(Math.atan2(offsetY, offsetX)) % 360);
    }

    public double getDegrees() {
        return drive.getPose().getRotation().getDegrees();
    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getCurrentPose() {
        return drive.getPose();
    }

    public ChassisSpeeds getChassisSpeeds() {
        return drive.getChassisSpeeds();
    }

    public Rotation2d getCurrentRotation() {
        return drive.getPose().getRotation();
    }

    public RobotConfig getRobotConfig() {
        return config;
    }

    public Pose2d getTargetPose() {
        return targetPose;
    }

    public double getTargetYaw() {
        return targetYaw;
    }

    public void setTargetPose(Pose2d newTarget) {
        targetPose = newTarget;
    }

    public Command setTargetPoseCommand(Pose2d newTarget) {
        return Commands.runOnce(() -> setTargetPose(newTarget));
    }

    public boolean atTargetPose() {
        Pose2d relative = getCurrentPose().relativeTo(targetPose);
        return atTargetX(relative) && atTargetY(relative) && atTargetTheta(relative);
    }

    public boolean atTargetPose(boolean hasDistance) {
        Pose2d relative = getCurrentPose().relativeTo(targetPose);
        return atTargetX(relative, hasDistance) && atTargetY(relative) && atTargetTheta(relative);
    }

    public boolean atTargetX() {
        Pose2d relative = getCurrentPose().relativeTo(targetPose);
        return Math.abs(relative.getX()) < VisionConstants.xTolerance;
    }

    public boolean atTargetX(Pose2d pose) {
        return Math.abs(pose.getX()) < VisionConstants.xTolerance;
    }

    public boolean atTargetX(boolean hasDistance) {
        Pose2d relative = getCurrentPose().relativeTo(targetPose);
        return hasDistance
                ? Math.abs(relative.getX()) < VisionConstants.xToleranceHasDistance
                : Math.abs(relative.getX()) < VisionConstants.xTolerance;
    }

    public boolean atTargetX(Pose2d pose, boolean hasDistance) {
        return hasDistance
                ? Math.abs(pose.getX()) < VisionConstants.xToleranceHasDistance
                : Math.abs(pose.getX()) < VisionConstants.xTolerance;
    }

    public boolean atTargetY() {
        Pose2d relative = getCurrentPose().relativeTo(targetPose);
        return Math.abs(relative.getY()) < VisionConstants.yTolerance;
    }

    public boolean atTargetY(Pose2d pose) {
        return Math.abs(pose.getY()) < VisionConstants.yTolerance;
    }

    public boolean atTargetY(boolean hasDistance) {
        Pose2d relative = getCurrentPose().relativeTo(targetPose);
        return hasDistance
                ? Math.abs(relative.getY()) < VisionConstants.yToleranceHasDistance
                : Math.abs(relative.getY()) < VisionConstants.yTolerance;
    }

    public boolean atTargetY(Pose2d pose, boolean hasDistance) {
        return hasDistance
                ? Math.abs(pose.getY()) < VisionConstants.yToleranceHasDistance
                : Math.abs(pose.getY()) < VisionConstants.yTolerance;
    }

    public boolean atTargetTheta() {
        Pose2d relative = getCurrentPose().relativeTo(targetPose);
        return Math.abs(relative.getRotation().getDegrees()) < VisionConstants.thetaTolerance;
    }

    public boolean atTargetTheta(Pose2d pose) {
        return Math.abs(pose.getRotation().getDegrees()) < VisionConstants.thetaTolerance;
    }

    /**
     * Resets the current pose to the specified pose. This should ONLY be called
     * when the robot's
     * position on the field is known, like at the beginning of a match.
     *
     * @param newPose new pose
     */
    public void setCurrentPose(Pose2d newPose) {
        drive.setPose(newPose);
        // drive.setGyroAdjustment(newPose.getRotation().getDegrees());
    }

    public Command setCurrentPoseCommand(Pose2d newPose) {
        return runOnce(() -> setCurrentPose(newPose));
    }

    /**
     * Resets the position on the field to 0,0 0-degrees, with forward being
     * downfield. This resets
     * what "forward" is for field oriented driving.
     */
    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }

    /**
     * Transforms a pose to the opposite alliance's coordinate system. (0,0) is
     * always on the right
     * corner of your alliance wall, so for 2023, the field elements are at
     * different coordinates for
     * each alliance.
     *
     * @param poseToFlip pose to transform to the other alliance
     * @return pose relative to the other alliance's coordinate system
     */
    private Pose2d flipAlliance(Pose2d poseToFlip) {
        return poseToFlip.relativeTo(VisionConstants.FLIPPING_POSE);
    }

    private Heading getHeading() {
        return new Heading(Timer.getFPGATimestamp(), getCurrentRotation());
    }
}

