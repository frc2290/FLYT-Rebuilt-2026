package frc.utils;

import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.defaultLoopDtSeconds;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.latencyCompensationSeconds;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.maxNewtonIterations;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.maxRecursiveIterations;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.maxValidDistanceMeters;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.minDerivativeDistanceDeltaMeters;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.minDistanceMeters;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.minDragConstantMagnitude;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.minLoopDtSeconds;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.newtonMinDerivativeMagnitude;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.newtonToleranceSeconds;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.recursiveDragConstant;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.tofDerivativeStepMeters;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.turretShooterOffsetX;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.turretShooterOffsetY;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootOnTheFly {
    public static ShootOnTheFly instance = null;

    private final LinearInterpolator shootAngleInterp = new LinearInterpolator();
    private final LinearInterpolator shootSpeedInterp = new LinearInterpolator();

    public record FullShooterParams(double speedMetersPerSecond, double hoodAngle, double timeOfFlight) {
    }

    private static class TargetKinematics {
        private final Translation2d toGoal;
        private final Translation2d totalShooterVelocity;
        private final Translation2d totalShooterAcceleration;

        private TargetKinematics(Translation2d toGoal, Translation2d totalShooterVelocity,
                Translation2d totalShooterAcceleration) {
            this.toGoal = toGoal;
            this.totalShooterVelocity = totalShooterVelocity;
            this.totalShooterAcceleration = totalShooterAcceleration;
        }
    }

    private static class ConvergenceResult {
        private final boolean converged;
        private final int iterationsUsed;
        private final double tof;
        private final double projectedDistance;
        private final Translation2d finalBallGoal;

        private ConvergenceResult(boolean converged, int iterationsUsed, double tof, double projectedDistance,
                Translation2d finalBallGoal) {
            this.converged = converged;
            this.iterationsUsed = iterationsUsed;
            this.tof = tof;
            this.projectedDistance = projectedDistance;
            this.finalBallGoal = finalBallGoal;
        }
    }

    private InterpolatingTreeMap<Double, FullShooterParams> shooterMap;
    private double prevVx = 0.0;
    private double prevVy = 0.0;
    private double prevOmega = 0.0;
    private boolean hasPreviousVelocityState = false;
    private double previousTofRecursive = -1.0;
    private double previousTofNewton = -1.0;

    public static FullShooterParams interpolateParams(FullShooterParams startValue, FullShooterParams endValue,
            double t) {
        return new FullShooterParams(
                MathUtil.interpolate(startValue.speedMetersPerSecond(), endValue.speedMetersPerSecond(), t),
                MathUtil.interpolate(startValue.hoodAngle(), endValue.hoodAngle(), t),
                MathUtil.interpolate(startValue.timeOfFlight(), endValue.timeOfFlight(), t));
    }

    public static class SOTFResult {
        public double yaw; // turretAngle
        public double pitch; // hoodAngle
        public double vel; // exitVelocity
        public double dist; // distance
        public boolean isValid;
        public double yawVelocityRadPerSec;
        public double yawAccelerationRadPerSec2;
        public double pitchVelocityDegPerSec;
        public double flywheelAccelerationMetersPerSec2;
        public double tof;

        public SOTFResult(double yaw, double pitch, double vel, double dist) {
            this(yaw, pitch, vel, dist, true, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        public SOTFResult(double yaw, double pitch, double vel, double dist, boolean isValid) {
            this(yaw, pitch, vel, dist, isValid, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        public SOTFResult(double yaw, double pitch, double vel, double dist, boolean isValid,
                double yawVelocityRadPerSec, double yawAccelerationRadPerSec2, double pitchVelocityDegPerSec,
                double flywheelAccelerationMetersPerSec2, double tof) {
            this.yaw = yaw;
            this.pitch = pitch;
            this.vel = vel;
            this.dist = dist;
            this.isValid = isValid;
            this.yawVelocityRadPerSec = yawVelocityRadPerSec;
            this.yawAccelerationRadPerSec2 = yawAccelerationRadPerSec2;
            this.pitchVelocityDegPerSec = pitchVelocityDegPerSec;
            this.flywheelAccelerationMetersPerSec2 = flywheelAccelerationMetersPerSec2;
            this.tof = tof;
        }

        public static SOTFResult invalid() {
            return new SOTFResult(0.0, 0.0, 0.0, 0.0, false, 0.0, 0.0, 0.0, 0.0, 0.0);
        }
    }

    public void addShootInterpData(InterpolatingTreeMap<Double, FullShooterParams> shooterMap) {
        this.shooterMap = shooterMap;
    }

    public void addShootAngleInterpData(double[][] data) {
        shootAngleInterp.build_table(data);
    }

    public double getShootAngleInterp(double dist) {
        return shootAngleInterp.getInterpolatedValue(dist);
    }

    public void addShootSpeedInterpData(double[][] data) {
        shootSpeedInterp.build_table(data);
    }

    public double getShootSpeedInterp(double dist) {
        return shootSpeedInterp.getInterpolatedValue(dist);
    }

    public SOTFResult calculateRecursiveTOF(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        return calculateRecursiveTOF(
                goalLocation,
                robotPose,
                robotSpeeds,
                new Rotation2d(),
                0.0,
                defaultLoopDtSeconds);
    }

    public SOTFResult calculateRecursiveTOF(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeeds,
            Rotation2d currentTurretAngle, double turretOmegaRadPerSecond, double dt) {
        if (!isShooterMapReady()) {
            previousTofRecursive = -1.0;
            return SOTFResult.invalid();
        }

        TargetKinematics targetKinematics =
                buildTargetKinematics(goalLocation, robotPose, robotSpeeds, currentTurretAngle, turretOmegaRadPerSecond, dt);
        if (targetKinematics == null) {
            previousTofRecursive = -1.0;
            return SOTFResult.invalid();
        }

        ConvergenceResult convergence =
                solveRecursiveTOF(targetKinematics.toGoal, targetKinematics.totalShooterVelocity, previousTofRecursive);
        if (!isConvergenceValid(convergence)) {
            previousTofRecursive = -1.0;
            return SOTFResult.invalid();
        }

        previousTofRecursive = convergence.tof;
        return buildFinalResult(convergence, targetKinematics);
    }

    public SOTFResult calculateNewtonTOF(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        return calculateNewtonTOF(
                goalLocation,
                robotPose,
                robotSpeeds,
                new Rotation2d(),
                0.0,
                defaultLoopDtSeconds);
    }

    public SOTFResult calculateNewtonTOF(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeeds,
            Rotation2d currentTurretAngle, double turretOmegaRadPerSecond, double dt) {
        if (!isShooterMapReady()) {
            previousTofNewton = -1.0;
            return SOTFResult.invalid();
        }

        TargetKinematics targetKinematics =
                buildTargetKinematics(goalLocation, robotPose, robotSpeeds, currentTurretAngle, turretOmegaRadPerSecond, dt);
        if (targetKinematics == null) {
            previousTofNewton = -1.0;
            return SOTFResult.invalid();
        }

        ConvergenceResult convergence =
                solveNewtonTOF(targetKinematics.toGoal, targetKinematics.totalShooterVelocity, previousTofNewton);
        if (!isConvergenceValid(convergence)) {
            previousTofNewton = -1.0;
            return SOTFResult.invalid();
        }

        previousTofNewton = convergence.tof;
        return buildFinalResult(convergence, targetKinematics);
    }

    public SOTFResult calculateTOF(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        return calculateTOF(goalLocation, robotPose, robotSpeeds, defaultLoopDtSeconds);
    }

    public SOTFResult calculateTOF(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeeds,
            double dt) {
        return calculateNewtonTOF(goalLocation, robotPose, robotSpeeds, new Rotation2d(), 0.0, dt);
    }

    public SOTFResult calculate(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeed) {
        return calculate(goalLocation, robotPose, robotSpeed, defaultLoopDtSeconds);
    }

    public SOTFResult calculate(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeed, double dt) {
        return calculateNewtonTOF(goalLocation, robotPose, robotSpeed, new Rotation2d(), 0.0, dt);
    }

    public static ShootOnTheFly getInstance() {
        if (instance == null) {
            instance = new ShootOnTheFly();
        }
        return instance;
    }

    private ShootOnTheFly() {
    }

    private TargetKinematics buildTargetKinematics(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeeds,
            Rotation2d currentTurretAngle, double turretOmegaRadPerSecond, double dt) {
        double loopDt = dt > minLoopDtSeconds ? dt : defaultLoopDtSeconds;

        double ax = 0.0;
        double ay = 0.0;
        double aOmega = 0.0;
        if (hasPreviousVelocityState) {
            ax = (robotSpeeds.vxMetersPerSecond - prevVx) / loopDt;
            ay = (robotSpeeds.vyMetersPerSecond - prevVy) / loopDt;
            aOmega = (robotSpeeds.omegaRadiansPerSecond - prevOmega) / loopDt;
        }

        prevVx = robotSpeeds.vxMetersPerSecond;
        prevVy = robotSpeeds.vyMetersPerSecond;
        prevOmega = robotSpeeds.omegaRadiansPerSecond;
        hasPreviousVelocityState = true;

        double latencySeconds = latencyCompensationSeconds;
        double dx = robotSpeeds.vxMetersPerSecond * latencySeconds + 0.5 * ax * latencySeconds * latencySeconds;
        double dy = robotSpeeds.vyMetersPerSecond * latencySeconds + 0.5 * ay * latencySeconds * latencySeconds;
        double dTheta = robotSpeeds.omegaRadiansPerSecond * latencySeconds
                + 0.5 * aOmega * latencySeconds * latencySeconds;

        Pose2d futureRobotPose = robotPose.exp(new Twist2d(dx, dy, dTheta));
        Translation2d futureRobotCenter = futureRobotPose.getTranslation();
        Translation2d fieldVelocity = getFieldRelativeVelocity(robotSpeeds, robotPose);

        Rotation2d absoluteTurretAngle = futureRobotPose.getRotation().plus(currentTurretAngle);
        Translation2d shooterOffsetField = new Translation2d(turretShooterOffsetX, turretShooterOffsetY)
                .rotateBy(absoluteTurretAngle);
        Translation2d futureShooterPos = futureRobotCenter.plus(shooterOffsetField);

        double totalOmega = robotSpeeds.omegaRadiansPerSecond + turretOmegaRadPerSecond;
        Translation2d tangentialVelocity = new Translation2d(
                -shooterOffsetField.getY() * totalOmega,
                shooterOffsetField.getX() * totalOmega);
        Translation2d totalShooterVelocity = fieldVelocity.plus(tangentialVelocity);

        // Field-frame shooter acceleration includes chassis linear acceleration and
        // rigid-body rotational terms from the turret/shooter offset.
        Translation2d fieldAcceleration = new Translation2d(ax, ay).rotateBy(robotPose.getRotation());
        Translation2d rotationalAcceleration = new Translation2d(
                -shooterOffsetField.getX() * totalOmega * totalOmega - shooterOffsetField.getY() * aOmega,
                -shooterOffsetField.getY() * totalOmega * totalOmega + shooterOffsetField.getX() * aOmega);
        Translation2d totalShooterAcceleration = fieldAcceleration.plus(rotationalAcceleration);

        Translation2d toGoal = goalLocation.minus(futureShooterPos);
        double initialDistance = toGoal.getNorm();
        if (!Double.isFinite(initialDistance) || initialDistance < minDistanceMeters) {
            return null;
        }

        return new TargetKinematics(toGoal, totalShooterVelocity, totalShooterAcceleration);
    }

    private ConvergenceResult solveRecursiveTOF(Translation2d toGoal, Translation2d totalShooterVelocity,
            double warmStartTof) {
        double initialDistance = toGoal.getNorm();
        double tof = warmStartTof > 0.0 ? warmStartTof : getInterpolatedTof(initialDistance);
        if (!Double.isFinite(tof) || tof <= 0.0) {
            return new ConvergenceResult(false, 0, tof, initialDistance, toGoal);
        }

        int iterationsUsed = 0;
        boolean converged = false;
        double projectedDistance = initialDistance;
        Translation2d finalBallGoal = toGoal;

        for (int i = 0; i < maxRecursiveIterations; i++) {
            iterationsUsed = i + 1;

            double alphaTof = getDragAdjustedTof(tof);
            Translation2d ballGoal = toGoal.minus(totalShooterVelocity.times(alphaTof));
            double distance = ballGoal.getNorm();
            if (!Double.isFinite(distance) || distance < minDistanceMeters) {
                break;
            }

            double lookupTof = getInterpolatedTof(distance);
            if (!Double.isFinite(lookupTof) || lookupTof <= 0.0) {
                break;
            }

            double f = lookupTof - tof;
            tof = lookupTof;
            finalBallGoal = ballGoal;
            projectedDistance = distance;

            if (Math.abs(f) < newtonToleranceSeconds) {
                converged = true;
                break;
            }
        }

        return new ConvergenceResult(converged, iterationsUsed, tof, projectedDistance, finalBallGoal);
    }

    private ConvergenceResult solveNewtonTOF(Translation2d toGoal, Translation2d totalShooterVelocity, double warmStartTof) {
        double initialDistance = toGoal.getNorm();
        double tof = warmStartTof > 0.0 ? warmStartTof : getInterpolatedTof(initialDistance);
        if (!Double.isFinite(tof) || tof <= 0.0) {
            return new ConvergenceResult(false, 0, tof, initialDistance, toGoal);
        }

        int iterationsUsed = 0;
        boolean converged = false;
        double projectedDistance = initialDistance;
        Translation2d finalBallGoal = toGoal;

        for (int i = 0; i < maxNewtonIterations; i++) {
            iterationsUsed = i + 1;

            double alphaTof = getDragAdjustedTof(tof);
            Translation2d ballGoal = toGoal.minus(totalShooterVelocity.times(alphaTof));
            double projDist = ballGoal.getNorm();
            if (!Double.isFinite(projDist) || projDist < minDistanceMeters) {
                break;
            }

            double lookupTof = getInterpolatedTof(projDist);
            if (!Double.isFinite(lookupTof) || lookupTof <= 0.0) {
                break;
            }

            double f = lookupTof - tof;
            finalBallGoal = ballGoal;
            projectedDistance = projDist;
            if (Math.abs(f) < newtonToleranceSeconds) {
                converged = true;
                break;
            }

            double dPrime = -(ballGoal.getX() * totalShooterVelocity.getX()
                    + ballGoal.getY() * totalShooterVelocity.getY()) / projDist;
            double gPrime = getTofDerivative(projDist);
            if (!Double.isFinite(gPrime)) {
                break;
            }

            double fPrime = gPrime * dPrime - 1.0;
            if (Math.abs(fPrime) < newtonMinDerivativeMagnitude) {
                tof = lookupTof;
                continue;
            }

            double updatedTOF = tof - (f / fPrime);
            if (!Double.isFinite(updatedTOF) || updatedTOF <= 0.0) {
                break;
            }
            tof = updatedTOF;
        }

        return new ConvergenceResult(converged, iterationsUsed, tof, projectedDistance, finalBallGoal);
    }

    private SOTFResult buildFinalResult(ConvergenceResult convergence, TargetKinematics kinematics) {
        if (!isConvergenceValid(convergence) || kinematics == null) {
            return SOTFResult.invalid();
        }

        FullShooterParams params = shooterMap.get(convergence.projectedDistance);
        if (params == null) {
            return SOTFResult.invalid();
        }
        Translation2d r = convergence.finalBallGoal;
        double dist = convergence.projectedDistance;
        if (!Double.isFinite(dist) || dist < minDistanceMeters) {
            return SOTFResult.invalid();
        }

        Translation2d shooterVelocity = kinematics.totalShooterVelocity;
        Translation2d shooterAcceleration = kinematics.totalShooterAcceleration;

        // Relative target kinematics in the shooter frame:
        // rDot = -v_shooter and rDDot = -a_shooter.
        double radialVelocity = -((r.getX() * shooterVelocity.getX()) + (r.getY() * shooterVelocity.getY())) / dist;
        double tangentialVelocity = -((r.getX() * shooterVelocity.getY()) - (r.getY() * shooterVelocity.getX())) / dist;
        double yawVelocity = tangentialVelocity / dist;

        double tangentialAcceleration =
                -((r.getX() * shooterAcceleration.getY()) - (r.getY() * shooterAcceleration.getX())) / dist;
        double yawAcceleration = (tangentialAcceleration - (2.0 * radialVelocity * yawVelocity)) / dist;

        double pitchSlope = getPitchDerivative(dist);
        double pitchVelocity = Double.isFinite(pitchSlope) ? pitchSlope * radialVelocity : 0.0;

        double speedSlope = getSpeedDerivative(dist);
        double flywheelAcceleration = Double.isFinite(speedSlope) ? speedSlope * radialVelocity : 0.0;

        Rotation2d solvedYaw = r.getAngle();

        return new SOTFResult(
                solvedYaw.getDegrees(),
                params.hoodAngle(),
                params.speedMetersPerSecond(),
                dist,
                true,
                yawVelocity,
                yawAcceleration,
                pitchVelocity,
                flywheelAcceleration,
                convergence.tof);
    }

    private boolean isConvergenceValid(ConvergenceResult convergence) {
        return convergence != null
                && convergence.converged
                && Double.isFinite(convergence.tof)
                && convergence.tof > 0.0
                && Double.isFinite(convergence.projectedDistance)
                && convergence.projectedDistance > 0.0
                && convergence.projectedDistance <= maxValidDistanceMeters;
    }

    private boolean isShooterMapReady() {
        return shooterMap != null && shooterMap.get(0.0) != null;
    }

    private double getInterpolatedTof(double distanceMeters) {
        FullShooterParams params = shooterMap.get(distanceMeters);
        return params != null ? params.timeOfFlight() : Double.NaN;
    }

    private double getTofDerivative(double distanceMeters) {
        double h = tofDerivativeStepMeters;
        double upperDist = distanceMeters + h;
        double lowerDist = Math.max(0.0, distanceMeters - h);
        double tHigh = getInterpolatedTof(upperDist);
        double tLow = getInterpolatedTof(lowerDist);
        if (!Double.isFinite(tHigh) || !Double.isFinite(tLow)) {
            return Double.NaN;
        }
        double deltaDist = upperDist - lowerDist;
        if (deltaDist <= minDerivativeDistanceDeltaMeters) {
            return Double.NaN;
        }
        return (tHigh - tLow) / deltaDist;
    }

    private double getPitchDerivative(double distanceMeters) {
        if (!shootAngleInterp.isInitialized()) {
            return Double.NaN;
        }
        double h = tofDerivativeStepMeters;
        double upperDist = distanceMeters + h;
        double lowerDist = Math.max(0.0, distanceMeters - h);
        double deltaDist = upperDist - lowerDist;
        if (deltaDist <= minDerivativeDistanceDeltaMeters) {
            return Double.NaN;
        }

        double pHigh = shootAngleInterp.getInterpolatedValue(upperDist);
        double pLow = shootAngleInterp.getInterpolatedValue(lowerDist);
        if (!Double.isFinite(pHigh) || !Double.isFinite(pLow)) {
            return Double.NaN;
        }

        return (pHigh - pLow) / deltaDist;
    }

    private double getSpeedDerivative(double distanceMeters) {
        if (!shootSpeedInterp.isInitialized()) {
            return Double.NaN;
        }
        double h = tofDerivativeStepMeters;
        double upperDist = distanceMeters + h;
        double lowerDist = Math.max(0.0, distanceMeters - h);
        double deltaDist = upperDist - lowerDist;
        if (deltaDist <= minDerivativeDistanceDeltaMeters) {
            return Double.NaN;
        }

        double sHigh = shootSpeedInterp.getInterpolatedValue(upperDist);
        double sLow = shootSpeedInterp.getInterpolatedValue(lowerDist);
        if (!Double.isFinite(sHigh) || !Double.isFinite(sLow)) {
            return Double.NaN;
        }

        return (sHigh - sLow) / deltaDist;
    }

    private double getDragAdjustedTof(double tof) {
        if (Math.abs(recursiveDragConstant) < minDragConstantMagnitude) {
            return tof;
        }
        return (1.0 - Math.exp(-recursiveDragConstant * tof)) / recursiveDragConstant;
    }

    private Translation2d getFieldRelativeVelocity(ChassisSpeeds robotSpeeds, Pose2d robotPose) {
        return new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond)
                .rotateBy(robotPose.getRotation());
    }

}
