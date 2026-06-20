package frc.utils;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Optional;
import java.util.Random;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.turret.TurretConstants;
import frc.utils.ShootOnTheFly.FullShooterParams;
import frc.utils.ShootOnTheFly.SOTFResult;
import frc.utils.ShootOnTheFly.ShooterParamMap;
import frc.utils.ShootOnTheFly.TargetTable;

class ShootOnTheFlyTest {
    private static final double DT_SECONDS = TurretConstants.SotfConstants.defaultLoopDtSeconds;
    private static final long SOTF_FUZZ_SEED = 20260620L;
    private static final int SOTF_FUZZ_ITERATIONS_PER_TABLE = 750;

    private ShootOnTheFly sotf;

    @BeforeEach
    void setup() {
        ShootOnTheFly.instance = null;
        sotf = ShootOnTheFly.getInstance();
        sotf.addShootInterpData(TurretConstants.HUB_MAP, TargetTable.HUB);
        sotf.addShootInterpData(TurretConstants.SHUTTLE_MAP, TargetTable.SHUTTLE);
        sotf.setCurrentTofTable(TargetTable.HUB);
    }

    private void injectSyntheticLut(ShooterParamMap map) {
        sotf.addShootInterpData(map, TargetTable.HUB);
        sotf.setCurrentTofTable(TargetTable.HUB);
    }

    @Test
    void shooterParamMapInterpolatesAndUsesZeroOutOfBoundsDerivative() {
        ShooterParamMap map = new ShooterParamMap(
            Pair.of(0.0, new FullShooterParams(10.0, 20.0, 0.5)),
            Pair.of(2.0, new FullShooterParams(14.0, 30.0, 1.5))
        );

        assertEquals(10.0, map.speedMetersPerSecond(-1.0), 1e-9);
        assertEquals(12.0, map.speedMetersPerSecond(1.0), 1e-9);
        assertEquals(14.0, map.speedMetersPerSecond(3.0), 1e-9);
        assertEquals(25.0, map.hoodAngle(1.0), 1e-9);
        assertEquals(1.0, map.timeOfFlight(1.0), 1e-9);

        FullShooterParams middleDerivative = map.derivative(1.0);
        FullShooterParams lowerDerivative = map.derivative(-1.0);
        FullShooterParams upperDerivative = map.derivative(3.0);

        assertEquals(2.0, middleDerivative.speedMetersPerSecond(), 1e-9);
        assertEquals(5.0, middleDerivative.hoodAngle(), 1e-9);
        assertEquals(0.5, middleDerivative.timeOfFlight(), 1e-9);
        assertEquals(new FullShooterParams(0.0, 0.0, 0.0), lowerDerivative);
        assertEquals(new FullShooterParams(0.0, 0.0, 0.0), upperDerivative);
    }

    @Test
    void stationaryShotProducesFinitePresentResult() {
        Translation2d goalLocation = new Translation2d(5.0, 0.0);
        Pose2d robotPose = new Pose2d();
        ChassisSpeeds robotSpeeds = new ChassisSpeeds();
        Rotation2d turretAngle = new Rotation2d();

        SOTFResult result = sotf.calculateNewtonTOF(
                goalLocation,
                robotPose,
                robotSpeeds,
                turretAngle,
                0.0,
                DT_SECONDS)
                .orElseThrow();

        assertTrue(Math.abs(result.yaw()) < 1.0);
        assertTrue(result.vel() > 0.0);
        assertTrue(result.pitch() > 0.0);
        assertTrue(Double.isFinite(result.dist()));
        assertTrue(Double.isFinite(result.tof()));
        assertTrue(result.tof() > 0.0);
    }

    @Test
    void fieldEdgeCasesNeverThrowAndReturnFiniteOrEmpty() {
        double maxSpeed = DriveConstants.maxSpeedMetersPerSec;
        double maxOmega = DriveConstants.maxAngularSpeed;
        Translation2d hubTarget = FieldConstants.Hub.topCenterPoint.toTranslation2d();
        Translation2d shuttleTarget = new Translation2d(
                FieldConstants.LinesVertical.allianceZone * 0.25,
                FieldConstants.fieldWidth * 5.5 / 7.0);

        for (TargetTable table : TargetTable.values()) {
            sotf.setCurrentTofTable(table);
            Translation2d primaryTarget = table == TargetTable.HUB ? hubTarget : shuttleTarget;

            calculateAndAssertFiniteOrEmpty(
                    "exact target table=" + table,
                    primaryTarget,
                    new Pose2d(primaryTarget, new Rotation2d()),
                    new ChassisSpeeds(),
                    new Rotation2d(),
                    0.0,
                    DT_SECONDS);
            calculateAndAssertFiniteOrEmpty(
                    "near target table=" + table,
                    primaryTarget,
                    new Pose2d(primaryTarget.minus(new Translation2d(1.0e-7, 0.0)), new Rotation2d()),
                    new ChassisSpeeds(),
                    new Rotation2d(),
                    0.0,
                    DT_SECONDS);
            calculateAndAssertFiniteOrEmpty(
                    "far target table=" + table,
                    new Translation2d(FieldConstants.fieldLength + 20.0, FieldConstants.fieldWidth + 20.0),
                    new Pose2d(),
                    new ChassisSpeeds(),
                    new Rotation2d(),
                    0.0,
                    DT_SECONDS);
            calculateAndAssertFiniteOrEmpty(
                    "max linear speed table=" + table,
                    primaryTarget,
                    new Pose2d(FieldConstants.fieldLength * 0.5, FieldConstants.fieldWidth * 0.5,
                            Rotation2d.fromDegrees(90.0)),
                    new ChassisSpeeds(maxSpeed, -maxSpeed, 0.0),
                    Rotation2d.fromDegrees(180.0),
                    0.0,
                    DT_SECONDS);
            calculateAndAssertFiniteOrEmpty(
                    "combined high speed table=" + table,
                    primaryTarget,
                    new Pose2d(-0.5, FieldConstants.fieldWidth + 0.5, Rotation2d.fromDegrees(-135.0)),
                    new ChassisSpeeds(maxSpeed * 1.5, maxSpeed * 1.5, maxOmega * 1.5),
                    Rotation2d.fromDegrees(720.0),
                    maxOmega * 2.0,
                    0.1);
            calculateAndAssertFiniteOrEmpty(
                    "reverse combined high speed table=" + table,
                    primaryTarget,
                    new Pose2d(FieldConstants.fieldLength + 0.5, -0.5, Rotation2d.fromDegrees(135.0)),
                    new ChassisSpeeds(-maxSpeed * 1.5, -maxSpeed * 1.5, -maxOmega * 1.5),
                    Rotation2d.fromDegrees(-720.0),
                    -maxOmega * 2.0,
                    1.0e-6);
        }
    }

    @Test
    void randomizedFieldFuzzNeverThrowsAndReturnsFiniteOrEmpty() {
        Random random = new Random(SOTF_FUZZ_SEED);
        double fieldMarginMeters = 1.0;
        double maxSpeed = DriveConstants.maxSpeedMetersPerSec;
        double maxOmega = DriveConstants.maxAngularSpeed;

        for (TargetTable table : TargetTable.values()) {
            sotf.setCurrentTofTable(table);
            for (int i = 0; i < SOTF_FUZZ_ITERATIONS_PER_TABLE; i++) {
                Pose2d robotPose = new Pose2d(
                        randomRange(random, -fieldMarginMeters, FieldConstants.fieldLength + fieldMarginMeters),
                        randomRange(random, -fieldMarginMeters, FieldConstants.fieldWidth + fieldMarginMeters),
                        Rotation2d.fromDegrees(randomRange(random, -720.0, 720.0)));
                Translation2d target = randomTarget(random, table, fieldMarginMeters);
                ChassisSpeeds robotSpeeds = new ChassisSpeeds(
                        randomRange(random, -maxSpeed * 1.5, maxSpeed * 1.5),
                        randomRange(random, -maxSpeed * 1.5, maxSpeed * 1.5),
                        randomRange(random, -maxOmega * 1.5, maxOmega * 1.5));
                Rotation2d turretAngle = Rotation2d.fromDegrees(randomRange(random, -720.0, 720.0));
                double turretOmegaRadPerSecond = randomRange(random, -maxOmega * 2.0, maxOmega * 2.0);
                double dt = randomDt(random);

                calculateAndAssertFiniteOrEmpty(
                        "fuzz table=" + table + " iteration=" + i,
                        target,
                        robotPose,
                        robotSpeeds,
                        turretAngle,
                        turretOmegaRadPerSecond,
                        dt);
            }
        }
    }

    @Test
    void movingCrossfieldShotCompensatesYaw() {
        Translation2d goalLocation = new Translation2d(5.0, 0.0);
        Pose2d robotPose = new Pose2d();
        ChassisSpeeds robotSpeeds = new ChassisSpeeds(0.0, 3.0, 0.0);
        Rotation2d turretAngle = new Rotation2d();

        SOTFResult newtonResult = sotf.calculateNewtonTOF(
                goalLocation,
                robotPose,
                robotSpeeds,
                turretAngle,
                0.0,
                DT_SECONDS)
                .orElseThrow();

        assertTrue(newtonResult.yaw() < -5.0);
        assertTrue(Double.isFinite(newtonResult.dist()));
    }

    @Test
    void targetOutsideMaximumDistanceIsInvalid() {
        Translation2d goalLocation = new Translation2d(15.0, 0.0);
        Optional<SOTFResult> newtonResult = sotf.calculateNewtonTOF(
                goalLocation,
                new Pose2d(),
                new ChassisSpeeds(),
                new Rotation2d(),
                0.0,
                DT_SECONDS);

        assertTrue(newtonResult.isEmpty());
    }

    @Test
    void extremeOutwardVelocityIsInvalid() {
        Translation2d goalLocation = new Translation2d(5.0, 0.0);
        ChassisSpeeds robotSpeeds = new ChassisSpeeds(-20.0, 0.0, 0.0);

        Optional<SOTFResult> newtonResult = sotf.calculateNewtonTOF(
                goalLocation,
                new Pose2d(),
                robotSpeeds,
                new Rotation2d(),
                0.0,
                DT_SECONDS);

        assertTrue(newtonResult.isEmpty());
    }

    @Test
    void derivativeBoundaryClampStaysFinite() {
        ShooterParamMap map = new ShooterParamMap(
            Pair.of(0.0, new FullShooterParams(10.0, 45.0, 0.05)),
            Pair.of(0.5, new FullShooterParams(10.0, 45.0, 0.50))
        );
        injectSyntheticLut(map);

        SOTFResult lowerResult = sotf.calculateNewtonTOF(
                new Translation2d(0.1505, 0.0),
                new Pose2d(),
                new ChassisSpeeds(),
                new Rotation2d(),
                0.0,
                DT_SECONDS)
                .orElseThrow();

        SOTFResult upperResult = sotf.calculateNewtonTOF(
                new Translation2d(0.6, 0.0),
                new Pose2d(),
                new ChassisSpeeds(),
                new Rotation2d(),
                0.0,
                DT_SECONDS)
                .orElseThrow();

        assertTrue(Double.isFinite(lowerResult.dist()));
        assertTrue(lowerResult.dist() < 0.5);
        assertTrue(Double.isFinite(upperResult.dist()));
        assertTrue(upperResult.dist() > 0.5);
    }

    @Test
    void flatSlopeLutConverges() {
        ShooterParamMap map = new ShooterParamMap(
            Pair.of(0.0, new FullShooterParams(10.0, 45.0, 1.0)),
            Pair.of(10.0, new FullShooterParams(10.0, 45.0, 1.0))
        );
        injectSyntheticLut(map);

        Optional<SOTFResult> result = sotf.calculateNewtonTOF(
                new Translation2d(5.0, 0.0),
                new Pose2d(),
                new ChassisSpeeds(),
                new Rotation2d(),
                0.0,
                DT_SECONDS);

        assertTrue(result.isPresent());
    }

    @Test
    void warmStartRecoversAfterTargetJump() {
        ShooterParamMap map = new ShooterParamMap(
            Pair.of(0.0, new FullShooterParams(10.0, 45.0, 0.20)),
            Pair.of(10.0, new FullShooterParams(10.0, 45.0, 1.50))
        );
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
                DT_SECONDS)
                .orElseThrow();

        assertTrue(recoveryResult.dist() > 7.0);
    }

    @Test
    @SuppressWarnings("unchecked")
    void oscillatingLutSolvesWithoutConstructionFailure() {
        Pair<Double, FullShooterParams>[] params = new Pair[21];
        for (int i = 0; i <= 20; i++) {
            double distance = i * 0.5;
            double tof = (i % 2 == 0) ? 0.25 : 5.00;
            params[i] = Pair.of(distance, new FullShooterParams(10.0, 45.0, tof));
        }
        ShooterParamMap map = new ShooterParamMap(params);
        injectSyntheticLut(map);

        Optional<SOTFResult> result = sotf.calculateNewtonTOF(
                new Translation2d(5.25, 0.0),
                new Pose2d(),
                new ChassisSpeeds(2.0, 0.0, 0.0),
                new Rotation2d(),
                0.0,
                DT_SECONDS);

        assertTrue(result.isPresent());
        SOTFResult value = result.orElseThrow();
        assertTrue(Double.isFinite(value.dist()));
        assertTrue(Double.isFinite(value.tof()));
    }

    @Test
    void extremeSpinStillProducesFiniteValidSolution() {
        ShooterParamMap map = new ShooterParamMap(
            Pair.of(0.0, new FullShooterParams(10.0, 45.0, 0.30)),
            Pair.of(10.0, new FullShooterParams(10.0, 45.0, 1.00))
        );
        injectSyntheticLut(map);

        SOTFResult result = sotf.calculateNewtonTOF(
                new Translation2d(5.0, 0.0),
                new Pose2d(),
                new ChassisSpeeds(),
                new Rotation2d(),
                10.0,
                DT_SECONDS)
                .orElseThrow();

        assertTrue(Double.isFinite(result.yaw()));
        assertTrue(Double.isFinite(result.dist()));
    }

    private Optional<SOTFResult> calculateAndAssertFiniteOrEmpty(
            String scenario,
            Translation2d goalLocation,
            Pose2d robotPose,
            ChassisSpeeds robotSpeeds,
            Rotation2d currentTurretAngle,
            double turretOmegaRadPerSecond,
            double dt) {
        Optional<SOTFResult> result = assertDoesNotThrow(
                () -> sotf.calculateNewtonTOF(
                        goalLocation,
                        robotPose,
                        robotSpeeds,
                        currentTurretAngle,
                        turretOmegaRadPerSecond,
                        dt),
                scenario);
        assertTrue(result != null, scenario + " returned null Optional");
        result.ifPresent(value -> assertFiniteResult(scenario, value));
        return result;
    }

    private void assertFiniteResult(String scenario, SOTFResult result) {
        assertTrue(Double.isFinite(result.yaw()), scenario + " yaw must be finite");
        assertTrue(Double.isFinite(result.pitch()), scenario + " pitch must be finite");
        assertTrue(Double.isFinite(result.vel()), scenario + " velocity must be finite");
        assertTrue(Double.isFinite(result.dist()), scenario + " distance must be finite");
        assertTrue(Double.isFinite(result.yawVelocityRadPerSec()), scenario + " yaw velocity must be finite");
        assertTrue(Double.isFinite(result.yawAccelerationRadPerSec2()), scenario + " yaw acceleration must be finite");
        assertTrue(Double.isFinite(result.pitchVelocityDegPerSec()), scenario + " pitch velocity must be finite");
        assertTrue(Double.isFinite(result.flywheelAccelerationMetersPerSec2()),
                scenario + " flywheel acceleration must be finite");
        assertTrue(Double.isFinite(result.tof()), scenario + " TOF must be finite");
        assertTrue(result.dist() > 0.0, scenario + " distance must be positive");
        assertTrue(result.dist() <= TurretConstants.SotfConstants.maxValidDistanceMeters + 1.0e-9,
                scenario + " distance must stay within valid SOTF range");
        assertTrue(result.tof() > 0.0, scenario + " TOF must be positive");
    }

    private Translation2d randomTarget(Random random, TargetTable table, double fieldMarginMeters) {
        int selector = random.nextInt(8);
        if (selector == 0) {
            return FieldConstants.Hub.topCenterPoint.toTranslation2d();
        }
        if (selector == 1) {
            return FieldConstants.Hub.oppTopCenterPoint.toTranslation2d();
        }
        if (selector == 2) {
            double shuttleY = random.nextBoolean()
                    ? FieldConstants.fieldWidth * 1.5 / 7.0
                    : FieldConstants.fieldWidth * 5.5 / 7.0;
            return new Translation2d(FieldConstants.LinesVertical.allianceZone * 0.25, shuttleY);
        }
        if (selector == 3) {
            return new Translation2d(0.0, 0.0);
        }
        if (selector == 4) {
            return new Translation2d(FieldConstants.fieldLength, FieldConstants.fieldWidth);
        }
        if (selector == 5 && table == TargetTable.HUB) {
            return new Translation2d(FieldConstants.LinesVertical.hubCenter, FieldConstants.fieldWidth / 2.0);
        }

        return new Translation2d(
                randomRange(random, -fieldMarginMeters, FieldConstants.fieldLength + fieldMarginMeters),
                randomRange(random, -fieldMarginMeters, FieldConstants.fieldWidth + fieldMarginMeters));
    }

    private double randomDt(Random random) {
        int selector = random.nextInt(5);
        if (selector == 0) return DT_SECONDS;
        if (selector == 1) return 1.0e-6;
        if (selector == 2) return 0.1;
        if (selector == 3) return 0.25;
        return randomRange(random, 1.0e-5, 0.25);
    }

    private double randomRange(Random random, double min, double max) {
        return min + random.nextDouble() * (max - min);
    }
}
