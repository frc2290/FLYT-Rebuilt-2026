package frc.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.turret.TurretConstants;
import frc.utils.ShootOnTheFly.FullShooterParams;
import frc.utils.ShootOnTheFly.SOTFResult;

class ShootOnTheFlyTest {
    private static final double DT_SECONDS = TurretConstants.SotfConstants.defaultLoopDtSeconds;

    private ShootOnTheFly sotf;

    @BeforeEach
    void setup() {
        ShootOnTheFly.instance = null;
        sotf = ShootOnTheFly.getInstance();
        sotf.addShootInterpData(TurretConstants.SHOOTER_MAP);
        sotf.addShootAngleInterpData(TurretConstants.turretHoodData);
        sotf.addShootSpeedInterpData(TurretConstants.turretRPMData);
    }

    private static InterpolatingTreeMap<Double, FullShooterParams> newShooterMap() {
        return new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShootOnTheFly::interpolateParams);
    }

    private void injectSyntheticLut(InterpolatingTreeMap<Double, FullShooterParams> map) {
        sotf.addShootInterpData(map);
    }

    @Test
    void stationaryShotNewtonAndRecursiveAreValidAndClose() {
        Translation2d goalLocation = new Translation2d(5.0, 0.0);
        Pose2d robotPose = new Pose2d();
        ChassisSpeeds robotSpeeds = new ChassisSpeeds();
        Rotation2d turretAngle = new Rotation2d();

        SOTFResult recursiveResult =
                sotf.calculateRecursiveTOF(goalLocation, robotPose, robotSpeeds, turretAngle, 0.0, DT_SECONDS);
        SOTFResult newtonResult =
                sotf.calculateNewtonTOF(goalLocation, robotPose, robotSpeeds, turretAngle, 0.0, DT_SECONDS);

        assertTrue(recursiveResult.isValid);
        assertTrue(newtonResult.isValid);
        assertTrue(Math.abs(newtonResult.yaw) < 1.0);
        assertTrue(newtonResult.vel > 0.0);
        assertTrue(newtonResult.pitch > 0.0);
        assertEquals(recursiveResult.yaw, newtonResult.yaw, 1.0);
        assertEquals(recursiveResult.dist, newtonResult.dist, 0.2);
    }

    @Test
    void movingCrossfieldShotCompensatesYaw() {
        Translation2d goalLocation = new Translation2d(5.0, 0.0);
        Pose2d robotPose = new Pose2d();
        ChassisSpeeds robotSpeeds = new ChassisSpeeds(0.0, 3.0, 0.0);
        Rotation2d turretAngle = new Rotation2d();

        SOTFResult recursiveResult =
                sotf.calculateRecursiveTOF(goalLocation, robotPose, robotSpeeds, turretAngle, 0.0, DT_SECONDS);
        SOTFResult newtonResult =
                sotf.calculateNewtonTOF(goalLocation, robotPose, robotSpeeds, turretAngle, 0.0, DT_SECONDS);

        assertTrue(recursiveResult.isValid);
        assertTrue(newtonResult.isValid);
        assertTrue(recursiveResult.yaw < -5.0);
        assertTrue(newtonResult.yaw < -5.0);
        assertEquals(recursiveResult.yaw, newtonResult.yaw, 3.0);
        assertEquals(recursiveResult.dist, newtonResult.dist, 0.5);
    }

    @Test
    void targetOutsideMaximumDistanceIsInvalid() {
        Translation2d goalLocation = new Translation2d(15.0, 0.0);
        SOTFResult newtonResult = sotf.calculateNewtonTOF(
                goalLocation,
                new Pose2d(),
                new ChassisSpeeds(),
                new Rotation2d(),
                0.0,
                DT_SECONDS);

        assertFalse(newtonResult.isValid);
        assertEquals(0.0, newtonResult.yaw, 1e-6);
        assertEquals(0.0, newtonResult.vel, 1e-6);
    }

    @Test
    void extremeOutwardVelocityIsInvalid() {
        Translation2d goalLocation = new Translation2d(5.0, 0.0);
        ChassisSpeeds robotSpeeds = new ChassisSpeeds(-20.0, 0.0, 0.0);

        SOTFResult newtonResult = sotf.calculateNewtonTOF(
                goalLocation,
                new Pose2d(),
                robotSpeeds,
                new Rotation2d(),
                0.0,
                DT_SECONDS);

        assertFalse(newtonResult.isValid);
    }

    @Test
    void defaultTofPathUsesNewtonSolver() {
        Translation2d goalLocation = new Translation2d(4.0, 1.0);
        Pose2d robotPose = new Pose2d(0.25, -0.2, Rotation2d.fromDegrees(15.0));
        ChassisSpeeds robotSpeeds = new ChassisSpeeds(0.4, 1.2, 0.5);

        SOTFResult explicitNewton =
                sotf.calculateNewtonTOF(goalLocation, robotPose, robotSpeeds, new Rotation2d(), 0.0, DT_SECONDS);
        SOTFResult defaultTof = sotf.calculateTOF(goalLocation, robotPose, robotSpeeds, DT_SECONDS);

        assertEquals(explicitNewton.isValid, defaultTof.isValid);
        assertEquals(explicitNewton.yaw, defaultTof.yaw, 1e-9);
        assertEquals(explicitNewton.pitch, defaultTof.pitch, 1e-9);
        assertEquals(explicitNewton.vel, defaultTof.vel, 1e-9);
        assertEquals(explicitNewton.dist, defaultTof.dist, 1e-9);
    }

    @Test
    void derivativeBoundaryClampStaysFinite() {
        InterpolatingTreeMap<Double, FullShooterParams> map = newShooterMap();
        map.put(0.0, new FullShooterParams(10.0, 45.0, 0.05));
        map.put(0.5, new FullShooterParams(10.0, 45.0, 0.50));
        injectSyntheticLut(map);

        Translation2d goalLocation = new Translation2d(0.1505, 0.0);
        SOTFResult result = sotf.calculateNewtonTOF(
                goalLocation,
                new Pose2d(),
                new ChassisSpeeds(),
                new Rotation2d(),
                0.0,
                DT_SECONDS);

        assertTrue(result.isValid);
        assertTrue(Double.isFinite(result.dist));
        assertTrue(result.dist < 0.5);
    }

    @Test
    void flatSlopeLutConverges() {
        InterpolatingTreeMap<Double, FullShooterParams> map = newShooterMap();
        map.put(0.0, new FullShooterParams(10.0, 45.0, 1.0));
        map.put(10.0, new FullShooterParams(10.0, 45.0, 1.0));
        injectSyntheticLut(map);

        SOTFResult result = sotf.calculateNewtonTOF(
                new Translation2d(5.0, 0.0),
                new Pose2d(),
                new ChassisSpeeds(),
                new Rotation2d(),
                0.0,
                DT_SECONDS);

        assertTrue(result.isValid);
    }

    @Test
    void warmStartRecoversAfterTargetJump() {
        InterpolatingTreeMap<Double, FullShooterParams> map = newShooterMap();
        map.put(0.0, new FullShooterParams(10.0, 45.0, 0.20));
        map.put(10.0, new FullShooterParams(10.0, 45.0, 1.50));
        injectSyntheticLut(map);

        sotf.calculateNewtonTOF(
                new Translation2d(2.0, 0.0),
                new Pose2d(),
                new ChassisSpeeds(),
                new Rotation2d(),
                0.0,
                DT_SECONDS);

        SOTFResult recoveryResult = sotf.calculateNewtonTOF(
                new Translation2d(8.0, 0.0),
                new Pose2d(),
                new ChassisSpeeds(),
                new Rotation2d(),
                0.0,
                DT_SECONDS);

        assertTrue(recoveryResult.isValid);
        assertTrue(recoveryResult.dist > 7.0);
    }

    @Test
    void oscillatingLutMarksInvalidWhenNotConverged() {
        InterpolatingTreeMap<Double, FullShooterParams> map = newShooterMap();
        for (int i = 0; i <= 20; i++) {
            double distance = i * 0.5;
            double tof = (i % 2 == 0) ? 0.25 : 5.00;
            map.put(distance, new FullShooterParams(10.0, 45.0, tof));
        }
        injectSyntheticLut(map);

        SOTFResult result = sotf.calculateNewtonTOF(
                new Translation2d(5.25, 0.0),
                new Pose2d(),
                new ChassisSpeeds(2.0, 0.0, 0.0),
                new Rotation2d(),
                0.0,
                DT_SECONDS);

        assertFalse(result.isValid);
    }

    @Test
    void extremeSpinStillProducesFiniteValidSolution() {
        InterpolatingTreeMap<Double, FullShooterParams> map = newShooterMap();
        map.put(0.0, new FullShooterParams(10.0, 45.0, 0.30));
        map.put(10.0, new FullShooterParams(10.0, 45.0, 1.00));
        injectSyntheticLut(map);

        SOTFResult result = sotf.calculateNewtonTOF(
                new Translation2d(5.0, 0.0),
                new Pose2d(),
                new ChassisSpeeds(),
                new Rotation2d(),
                10.0,
                DT_SECONDS);

        assertTrue(result.isValid);
        assertTrue(Double.isFinite(result.yaw));
        assertTrue(Double.isFinite(result.dist));
    }
}
