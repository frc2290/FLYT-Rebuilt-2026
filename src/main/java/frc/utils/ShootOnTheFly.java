package frc.utils;

import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.latencyCompensationSeconds;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.maxNewtonIterations;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.maxValidDistanceMeters;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.minDistanceMeters;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.newtonMinDerivativeMagnitude;
import static frc.robot.subsystems.turret.TurretConstants.SotfConstants.newtonToleranceSeconds;

import java.util.Map;
import java.util.Optional;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootOnTheFly {
    public static ShootOnTheFly instance = null;

    private ShooterParamMap hubMap;
    private ShooterParamMap shuttleMap;
    // private double prevVx = 0.0;
    // private double prevVy = 0.0;
    // private double prevOmega = 0.0;
    // private boolean hasPreviousVelocityState = false;
    private double previousTofNewton = -1.0;
    private TargetTable currentTargetTable = TargetTable.HUB;

    public enum TargetTable {
        HUB,
        SHUTTLE
    }

    public record FullShooterParams(double speedMetersPerSecond, double hoodAngle, double timeOfFlight) {
        public static FullShooterParams interpolate(FullShooterParams startValue, FullShooterParams endValue,
            double t) {
            return new FullShooterParams(
                MathUtil.interpolate(startValue.speedMetersPerSecond(), endValue.speedMetersPerSecond(), t),
                MathUtil.interpolate(startValue.hoodAngle(), endValue.hoodAngle(), t),
                MathUtil.interpolate(startValue.timeOfFlight(), endValue.timeOfFlight(), t));
        }

        public static FullShooterParams derive(FullShooterParams upper, FullShooterParams lower, double delta) {
            return new FullShooterParams(
                (upper.speedMetersPerSecond() - lower.speedMetersPerSecond()) / delta,
                (upper.hoodAngle() - lower.hoodAngle()) / delta,
                (upper.timeOfFlight() - lower.timeOfFlight()) / delta
            );
        }
    }

    public static class ShooterParamMap {
        private final TreeMap<Double, FullShooterParams> m_map;
        // this is safe because we are only iterating over `params`
        @SafeVarargs
        public ShooterParamMap(Pair<Double, FullShooterParams>... params) {
            m_map = new TreeMap<>();
            for (Pair<Double, FullShooterParams> param : params) {
                m_map.put(param.getFirst(), param.getSecond());
            }
        }

        // basically copy-pasted from InterpolatingTreeMap
        public FullShooterParams get(Double key) {
            FullShooterParams val = m_map.get(key);
            if (val != null) return val;
            Double ceilingKey = m_map.ceilingKey(key);
            Double floorKey = m_map.floorKey(key);

            if (ceilingKey == null && floorKey == null) return null;
            if (ceilingKey == null) return m_map.get(floorKey);
            if (floorKey == null) return m_map.get(ceilingKey);

            FullShooterParams floor = m_map.get(floorKey);
            FullShooterParams ceiling = m_map.get(ceilingKey);

            return FullShooterParams.interpolate(
                floor, ceiling, MathUtil.inverseInterpolate(floorKey, ceilingKey, key));
        }

        public FullShooterParams derivative(double distance) {
            Map.Entry<Double,ShootOnTheFly.FullShooterParams> upper = m_map.higherEntry(distance);
            Map.Entry<Double,ShootOnTheFly.FullShooterParams> lower = m_map.floorEntry(distance);
            double delta = upper.getKey() - lower.getKey();
            return FullShooterParams.derive(upper.getValue(), lower.getValue(), delta);
        }

        public double speedMetersPerSecond(double distance) {
            return get(distance).speedMetersPerSecond();
        }

        public double hoodAngle(double distance) {
            return get(distance).hoodAngle();
        }

        public double timeOfFlight(double distance) {
            return get(distance).timeOfFlight();
        }
    }

    private static record TargetKinematics (
        Translation2d toGoal,
        Translation2d totalShooterVelocity,
        Translation2d totalShooterAcceleration
    ) {}

    private static record ConvergenceResult(
        boolean converged,
        double tof,
        double projectedDistance,
        Translation2d finalBallGoal
    ) {}

    public static record SOTFResult (
        double yaw, // turretAngle
        double pitch, // hoodAngle
        double vel, // exitVelocity
        double dist, // distance
        double yawVelocityRadPerSec,
        double yawAccelerationRadPerSec2,
        double pitchVelocityDegPerSec,
        double flywheelAccelerationMetersPerSec2,
        double tof
    ) {}

    // constructor stuff
    private ShootOnTheFly() {}

    public static ShootOnTheFly getInstance() {
        if (instance == null) {
            instance = new ShootOnTheFly();
        }
        return instance;
    }

    public void addShootInterpData(ShooterParamMap shotMap, TargetTable targetTable) {
        switch (targetTable) {
            case HUB:
                this.hubMap = shotMap;
                break;
            case SHUTTLE:
                this.shuttleMap = shotMap;
                break;
        }
    }

    public Optional<SOTFResult> calculateNewtonTOF(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeeds,
            Rotation2d currentTurretAngle, double turretOmegaRadPerSecond, double dt) {
        if (!isShooterMapReady()) {
            previousTofNewton = -1.0;
            return Optional.empty();
        }

        TargetKinematics targetKinematics =
                buildTargetKinematics(goalLocation, robotPose, robotSpeeds, currentTurretAngle, turretOmegaRadPerSecond, dt);
        if (targetKinematics == null) {
            previousTofNewton = -1.0;
            return Optional.empty();
        }

        ConvergenceResult convergence =
                solveNewtonTOF(targetKinematics.toGoal, targetKinematics.totalShooterVelocity, previousTofNewton);
        if (!isConvergenceValid(convergence)) {
            previousTofNewton = -1.0;
            return Optional.empty();
        }

        previousTofNewton = convergence.tof;
        return buildFinalResult(convergence, targetKinematics);
    }

    private TargetKinematics buildTargetKinematics(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeeds,
            Rotation2d currentTurretAngle, double turretOmegaRadPerSecond, double dt) {
        // double loopDt = dt > minLoopDtSeconds ? dt : defaultLoopDtSeconds;
        double vx = robotSpeeds.vxMetersPerSecond;
        double vy = robotSpeeds.vyMetersPerSecond;
        double vOmega = robotSpeeds.omegaRadiansPerSecond;

        double ax = 0.0;
        double ay = 0.0;
        double aOmega = 0.0;
        // not using accel compensation for now, it hurts more than it helps
        // if (hasPreviousVelocityState) {
        //     ax = (vx - prevVx) / loopDt;
        //     ay = (vy - prevVy) / loopDt;
        //     aOmega = (vOmega - prevOmega) / loopDt;
        // }

        // prevVx = vx;
        // prevVy = vy;
        // prevOmega = vOmega;
        // hasPreviousVelocityState = true;

        double latencySeconds = latencyCompensationSeconds;
        double dx = vx * latencySeconds + 0.5 * ax * latencySeconds * latencySeconds;
        double dy = vy * latencySeconds + 0.5 * ay * latencySeconds * latencySeconds;
        double dTheta = vOmega * latencySeconds
                + 0.5 * aOmega * latencySeconds * latencySeconds;

        Pose2d futureRobotPose = robotPose.exp(new Twist2d(dx, dy, dTheta));
        Translation2d futureRobotCenter = futureRobotPose.getTranslation();
        Translation2d fieldVelocity = new Translation2d(vx, vy)
                .rotateBy(robotPose.getRotation());

        // Field-frame shooter acceleration includes chassis linear acceleration and
        // rigid-body rotational terms from the turret/shooter offset.
        Translation2d fieldAcceleration = new Translation2d(ax, ay).rotateBy(robotPose.getRotation());

        Translation2d toGoal = goalLocation.minus(futureRobotCenter);
        double initialDistance = toGoal.getNorm();
        if (!Double.isFinite(initialDistance) || initialDistance < minDistanceMeters) {
            return null;
        }

        return new TargetKinematics(toGoal, fieldVelocity, fieldAcceleration);
    }

    private ConvergenceResult solveNewtonTOF(Translation2d toGoal, Translation2d totalShooterVelocity, double warmStartTof) {
        ShooterParamMap paramMap = getParamMap();
        double initialDistance = toGoal.getNorm();
        double tof = warmStartTof > 0.0 ? warmStartTof : paramMap.timeOfFlight(initialDistance);
        if (!Double.isFinite(tof) || tof <= 0.0) {
            return new ConvergenceResult(false, tof, initialDistance, toGoal);
        }

        boolean converged = false;
        double projectedDistance = initialDistance;
        Translation2d finalBallGoal = toGoal;

        for (int i = 0; i < maxNewtonIterations; i++) {
            Translation2d ballGoal = toGoal.minus(totalShooterVelocity.times(tof));
            double projDist = ballGoal.getNorm();
            if (!Double.isFinite(projDist) || projDist < minDistanceMeters) {
                break;
            }

            double lookupTof = paramMap.timeOfFlight(projDist);
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
            double gPrime = paramMap.derivative(projDist).timeOfFlight();
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

        return new ConvergenceResult(converged, tof, projectedDistance, finalBallGoal);
    }

    private Optional<SOTFResult> buildFinalResult(ConvergenceResult convergence, TargetKinematics kinematics) {
        ShooterParamMap paramMap = getParamMap();
        FullShooterParams params = paramMap.get(convergence.projectedDistance);
        if (params == null) return Optional.empty();

        Translation2d r = convergence.finalBallGoal;
        double dist = convergence.projectedDistance;
        if (!Double.isFinite(dist) || dist < minDistanceMeters) return Optional.empty();

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

        FullShooterParams derivative = paramMap.derivative(dist);
        double pitchSlope = derivative.hoodAngle();
        double pitchVelocity = Double.isFinite(pitchSlope) ? pitchSlope * radialVelocity : 0.0;

        double speedSlope = derivative.speedMetersPerSecond();
        double flywheelAcceleration = Double.isFinite(speedSlope) ? speedSlope * radialVelocity : 0.0;

        Rotation2d solvedYaw = r.getAngle();

        return Optional.of(new SOTFResult(
                solvedYaw.getDegrees(),
                params.hoodAngle(),
                params.speedMetersPerSecond(),
                dist,
                yawVelocity,
                yawAcceleration,
                pitchVelocity,
                flywheelAcceleration,
                convergence.tof));
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
        switch (currentTargetTable) {
            case HUB:
                return hubMap != null && hubMap.get(0.0) != null;
            case SHUTTLE:
                return shuttleMap != null && shuttleMap.get(0.0) != null;
            default:
                return false;
        }
    }

    public void setCurrentTofTable(TargetTable targetTable) {
        this.currentTargetTable = targetTable;
    }

    public TargetTable getCurrentTofTable() {
        return this.currentTargetTable;
    }

    public ShooterParamMap getParamMap() {
        return currentTargetTable == TargetTable.HUB ? hubMap : shuttleMap;
    }
}
