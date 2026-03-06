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

import static frc.robot.Constants.VisionConstants.kVisionCameraStdDevFactor;
import static frc.robot.Constants.VisionConstants.kVisionFinalThetaStdDevFallback;
import static frc.robot.Constants.VisionConstants.kVisionFinalThetaStdDevFloor;
import static frc.robot.Constants.VisionConstants.kVisionFinalXyStdDevFloor;
import static frc.robot.Constants.VisionConstants.kVisionInvalidStdDev;
import static frc.robot.Constants.VisionConstants.kVisionRejectVarianceThreshold;
import static frc.robot.Constants.VisionConstants.kVisionRotationVarianceFusionCutoff;
import static frc.robot.Constants.VisionConstants.kVisionStdDevDistanceExponent;
import static frc.robot.Constants.VisionConstants.kVisionStdDevMin;
import static frc.robot.Constants.VisionConstants.kVisionStdDevTagCountExponent;
import static frc.robot.Constants.VisionConstants.kVisionThetaStdDevCoefficient;
import static frc.robot.Constants.VisionConstants.kVisionXyStdDevCoefficient;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;

public class VisionPoseFuser {
    // Record to cleanly return the results back to the subsystem.
    public record FusedVisionUpdate(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {}

    /**
     * Fuses multiple PhotonVision estimates into a single highly-accurate pose.
     *
     * @param estimates List of EstimatedRobotPoses from your cameras.
     * @return A FusedVisionUpdate, or Optional.empty() if all frames were rejected.
     */
    public static Optional<FusedVisionUpdate> fuse(
            List<EstimatedRobotPose> estimates) {
        if (estimates == null || estimates.isEmpty()) {
            return Optional.empty();
        }

        double targetTime = estimates.stream().mapToDouble(est -> est.timestampSeconds).max().orElse(0.0);

        double sumWeightX = 0.0;
        double sumWeightedX = 0.0;
        double sumWeightY = 0.0;
        double sumWeightedY = 0.0;
        double sumWeightRot = 0.0;
        double sumWeightedCos = 0.0;
        double sumWeightedSin = 0.0;

        for (EstimatedRobotPose est : estimates) {
            // 1. Calculate standard deviations via private helper.
            Matrix<N3, N1> stdDevs = calculateStdDevs(est);
            double varX = Math.pow(stdDevs.get(0, 0), 2);
            double varY = Math.pow(stdDevs.get(1, 0), 2);
            double varRot = Math.pow(stdDevs.get(2, 0), 2);

            if (varX >= kVisionRejectVarianceThreshold) {
                continue;
            }

            Pose2d visionPose = est.estimatedPose.toPose2d();

            double wX = 1.0 / varX;
            double wY = 1.0 / varY;

            sumWeightX += wX;
            sumWeightedX += visionPose.getX() * wX;
            sumWeightY += wY;
            sumWeightedY += visionPose.getY() * wY;

            if (varRot < kVisionRotationVarianceFusionCutoff) {
                double wRot = 1.0 / varRot;
                sumWeightRot += wRot;
                sumWeightedCos += visionPose.getRotation().getCos() * wRot;
                sumWeightedSin += visionPose.getRotation().getSin() * wRot;
            }
        }

        if (sumWeightX == 0.0) {
            return Optional.empty();
        }

        Rotation2d fusedRot = (sumWeightRot > 0.0)
                ? new Rotation2d(sumWeightedCos / sumWeightRot, sumWeightedSin / sumWeightRot)
                : estimates.get(0).estimatedPose.toPose2d().getRotation();

        Pose2d finalPose = new Pose2d(sumWeightedX / sumWeightX, sumWeightedY / sumWeightY, fusedRot);

        Matrix<N3, N1> finalStdDevs = VecBuilder.fill(
                Math.max(Math.sqrt(1.0 / sumWeightX), kVisionFinalXyStdDevFloor),
                Math.max(Math.sqrt(1.0 / sumWeightY), kVisionFinalXyStdDevFloor),
                Math.max(
                        sumWeightRot > 0.0
                                ? Math.sqrt(1.0 / sumWeightRot)
                                : kVisionFinalThetaStdDevFallback,
                        kVisionFinalThetaStdDevFloor));

        return Optional.of(new FusedVisionUpdate(finalPose, targetTime, finalStdDevs));
    }

    /** Calculates dynamic standard deviations using Team 6328's polynomial formula. */
    private static Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose est) {
        var targets = est.targetsUsed;
        int numTags = targets.size();

        if (numTags == 0) {
            return VecBuilder.fill(kVisionInvalidStdDev, kVisionInvalidStdDev, kVisionInvalidStdDev);
        }

        double avgDist = 0.0;
        for (var target : targets) {
            avgDist += target.getBestCameraToTarget().getTranslation().getNorm();
        }
        avgDist /= numTags;

        // 6328 formula.
        double xyStd = kVisionXyStdDevCoefficient
                * Math.pow(avgDist, kVisionStdDevDistanceExponent)
                / Math.pow(numTags, kVisionStdDevTagCountExponent)
                * kVisionCameraStdDevFactor;
        double rotStd = kVisionThetaStdDevCoefficient
                * Math.pow(avgDist, kVisionStdDevDistanceExponent)
                / Math.pow(numTags, kVisionStdDevTagCountExponent)
                * kVisionCameraStdDevFactor;

        xyStd = Math.max(xyStd, kVisionStdDevMin);
        rotStd = Math.max(rotStd, kVisionStdDevMin);

        return VecBuilder.fill(xyStd, xyStd, rotStd);
    }
}
