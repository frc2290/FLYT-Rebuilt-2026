package frc.robot.subsystems.turret;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.utils.ShootOnTheFly;
import frc.utils.ShootOnTheFly.FullShooterParams;

public class TurretConstants {
    // --- CAN IDs ---
    public static final int turretCanId = 20;
    public static final int hoodCanId = 21;
    public static final int flywheel1CanId = 22;
    public static final int flywheel2CanId = 23;

    // --- Motor Configurations ---
    public static final boolean turretIsInverted = false;
    public static final boolean hoodIsInverted = false;
    public static final boolean flywheelIsInverted = false;
    public static final int turretMotorCurrent = 60;
    public static final int hoodMotorCurrent = 40;
    public static final int flywheelMotorCurrent = 60;
    public static final int turretCurrentLimit = 50;

    // --- Physical Dimensions & Mechanics ---
    public static final double turretHeight = inchesToMeters(24);
    public static final double shooterWheelDiameterMeters = 3.0 * 0.0254;
    public static final double rangeTurret = 2.0;
    public static final double numTeethTurret = 240.0;
    public static final double numTeethPulley2 = 27.0; // prev 25
    public static final double numTeethPulley1 = 26.0;
    public static final double numTeethMotor = 11.0;
    public static final double turretTurnReduction = 7 / 1;
    public static final double turretShootReduction = 20.0 / 40.0;
    public static final double hoodMotorToEncoderReduction = (33.0 / 11.0) * (23.0);
    public static final double hoodEncoderToHoodReduction = 1.0;
    public static final double turretHoodReduction = hoodMotorToEncoderReduction * hoodEncoderToHoodReduction;

    // --- Encoder & Conversion Factors ---
    public static final boolean hoodEncoderInverted = false;
    public static final double hoodEncoderZeroOffset = 0.8;
    public static final double hoodAngleOffset = 248.6 + 6;
    public static final double hoodShotAngleToleranceDeg = 2.0;
    public static final double hoodPitchCalibrationGain = -0.9105;
    public static final double hoodPitchCalibrationOffset = 82.1431;
    public static final double flywheelSpeedCalibrationGain = 0.8751;
    public static final double flywheelSpeedCalibrationOffset = 0.1592;

    // Spark relative encoder is on the turret turn motor.
    // Position factor: motor rotations -> turret mechanism degrees.
    public static final double turretEncoderPositionFactor = (numTeethMotor / numTeethTurret) * 360.0;
    // Velocity factor: motor RPM -> turret mechanism deg/s.
    public static final double turretEncoderVelocityFactor = turretEncoderPositionFactor / 60.0;
    public static final double hoodEncoderPositionFactor = (1.0 / hoodEncoderToHoodReduction) * 360.0;
    public static final double hoodEncoderVelocityFactor = hoodEncoderPositionFactor / 60.0;
    // Motor rotations -> ball travel meters (hooded shooter is 2:1 wheel-to-ball speed)
    public static final double flywheelEncoderPositionFactor =
            (1.0 / turretShootReduction) * (Math.PI * shooterWheelDiameterMeters) / 2.0;
    // Motor RPM -> ball speed m/s
    public static final double flywheelEncoderVelocityFactor = flywheelEncoderPositionFactor / 60.0;

    // --- Control Loop Constants (PID & Feedforward) ---
    public static final double turretTurnP = 1.0;
    public static final double turretTurnI = 0.0;
    public static final double turretTurnD = 0.0;
    public static final double turretff = 0;
    public static final DCMotor turretGearbox = DCMotor.getNeoVortex(1);
    public static final double turretFreeSpeedRPM =
            Units.radiansPerSecondToRotationsPerMinute(turretGearbox.freeSpeedRadPerSec);
    public static final double turretTheoreticalKv =
            12.0 / (turretFreeSpeedRPM * turretEncoderVelocityFactor);
    public static final double turretKp = 0.25;
    public static final double turretKi = 0.0;
    public static final double turretKd = 2.0;

    public static final double hoodff = 0;
    public static final DCMotor hoodGearbox = DCMotor.getNEO(1);
    public static final double hoodFreeSpeedRPM =
            Units.radiansPerSecondToRotationsPerMinute(hoodGearbox.freeSpeedRadPerSec);
    public static final double hoodKv = 12.0
            / ((hoodFreeSpeedRPM / hoodMotorToEncoderReduction) * hoodEncoderVelocityFactor);
    public static final double hoodKp = 0.025;
    public static final double hoodKi = 0.0;
    public static final double hoodKd = 0.0;

    public static final double shooterff = 0;
    public static final DCMotor flywheelGearbox = DCMotor.getNeoVortex(1);
    public static final double flywheelFreeSpeedRPM =
            Units.radiansPerSecondToRotationsPerMinute(flywheelGearbox.freeSpeedRadPerSec);
    public static final double flywheelKv = 0.4567;
    public static final double flywheelKs = 0.10545;
            //12 / (flywheelFreeSpeedRPM * flywheelEncoderVelocityFactor);
    public static final double flywheelKp = 0.2;
    public static final double flywheelKi = 0.0015;
    public static final double flywheelKd = 0.0;
    public static final double flywheelReadyRatio = 0.8;

    // --- Simulation Constants ---
    public static final double turretTurnSimP = 1.0;
    public static final double turretTurnSimI = 0.0;
    public static final double turretTurnSimD = 0.0;
    public static final double turretShootSimP = 1.0;
    public static final double turretShootSimI = 0.0;
    public static final double turretShootSimD = 0.0;
    public static final double turretHoodSimP = 1.0;
    public static final double turretHoodSimI = 0.0;
    public static final double turretHoodSimD = 0.0;

    // --- Subsystem Behaviors & Lookup Tables ---
    public static final InterpolatingTreeMap<Double, FullShooterParams> HUB_MAP = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), ShootOnTheFly::interpolateParams);
    static {
        HUB_MAP.put(0.0000, new FullShooterParams(6.18892637764662, 78.4999998500325, 1.04457571009897));
        HUB_MAP.put(1.0000, new FullShooterParams(6.20455137764662, 78.4999998500325, 1.04535009364389));
        HUB_MAP.put(1.5000, new FullShooterParams(6.56817351130289, 76.7295918970408, 1.04541935792468));
        HUB_MAP.put(2.0000, new FullShooterParams(6.74373767055121, 71.9781067444403, 1.04850462684171));
        HUB_MAP.put(2.5000, new FullShooterParams(7.0189807727238, 66.5389877430021, 1.0584691382551));
        HUB_MAP.put(3.0000, new FullShooterParams(7.34038047733374, 65.1221142555942, 1.05926550869139));
        HUB_MAP.put(3.5000, new FullShooterParams(7.54246497606623, 61.5751894715459, 1.05833832375859));
        HUB_MAP.put(4.0000, new FullShooterParams(7.7784859882, 58.3169774143067, 1.06361579174373));
        HUB_MAP.put(4.5000, new FullShooterParams(8.05588603104712, 55.2609703970057, 1.07582767250998));
        HUB_MAP.put(5.0000, new FullShooterParams(8.11075294485546, 53.083791108088, 1.08280007602822));
        HUB_MAP.put(5.5000, new FullShooterParams(8.49551412845361, 50.8141158294944, 1.08199125216117));
        HUB_MAP.put(6.0000, new FullShooterParams(8.79745832489632, 48.2447967540909, 1.08682115550695));
        HUB_MAP.put(14.0000, new FullShooterParams(12.9305491077761, 20.003119468689, 1.16507580482266));
    };

    public static final InterpolatingTreeMap<Double, FullShooterParams> SHUTTLE_MAP = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), ShootOnTheFly::interpolateParams);
    static {
        SHUTTLE_MAP.put(1.0000, new FullShooterParams(3.733813, 70.353757, 0.820045));
        SHUTTLE_MAP.put(1.5000, new FullShooterParams(4.439007, 67.703245, 0.926503));
        SHUTTLE_MAP.put(2.0000, new FullShooterParams(5.080442, 66.191789, 1.025526));
        SHUTTLE_MAP.put(2.5000, new FullShooterParams(5.681886, 65.314788, 1.120516));
        SHUTTLE_MAP.put(3.0000, new FullShooterParams(6.202885, 64.335025, 1.200103));
        SHUTTLE_MAP.put(3.5000, new FullShooterParams(6.693418, 63.552581, 1.275000));
        SHUTTLE_MAP.put(4.0000, new FullShooterParams(7.100870, 62.380226, 1.331092));
        SHUTTLE_MAP.put(4.5000, new FullShooterParams(7.486399, 61.297752, 1.382833));
        SHUTTLE_MAP.put(5.0000, new FullShooterParams(7.804552, 59.814176, 1.417331));
        SHUTTLE_MAP.put(5.5000, new FullShooterParams(8.111135, 58.387945, 1.448561));
        SHUTTLE_MAP.put(6.0000, new FullShooterParams(8.377355, 56.663147, 1.467243));
        SHUTTLE_MAP.put(6.5000, new FullShooterParams(8.642164, 54.991262, 1.483944));
        SHUTTLE_MAP.put(7.0000, new FullShooterParams(8.891982, 53.160483, 1.493316));
        SHUTTLE_MAP.put(7.5000, new FullShooterParams(9.147452, 51.391559, 1.501749));
        SHUTTLE_MAP.put(8.0000, new FullShooterParams(9.403778, 49.583392, 1.506779));
        SHUTTLE_MAP.put(8.5000, new FullShooterParams(9.668963, 47.846504, 1.511497));
        SHUTTLE_MAP.put(9.0000, new FullShooterParams(9.941604, 46.141911, 1.515006));
        SHUTTLE_MAP.put(9.5000, new FullShooterParams(10.223680, 44.511431, 1.518485));
        SHUTTLE_MAP.put(10.0000, new FullShooterParams(10.514643, 42.941886, 1.521684));
        SHUTTLE_MAP.put(10.5000, new FullShooterParams(10.814504, 41.443072, 1.524938));
        SHUTTLE_MAP.put(11.0000, new FullShooterParams(11.122853, 40.009851, 1.528203));
        SHUTTLE_MAP.put(11.5000, new FullShooterParams(11.439309, 38.643092, 1.531586));
        SHUTTLE_MAP.put(12.0000, new FullShooterParams(11.763453, 37.339282, 1.535081));
        SHUTTLE_MAP.put(12.5000, new FullShooterParams(12.094884, 36.095546, 1.538699));
        SHUTTLE_MAP.put(13.0000, new FullShooterParams(12.433209, 34.909118, 1.542449));
        SHUTTLE_MAP.put(13.5000, new FullShooterParams(12.778068, 33.776872, 1.546332));
        SHUTTLE_MAP.put(14.0000, new FullShooterParams(13.129125, 32.695801, 1.550345));
        SHUTTLE_MAP.put(14.5000, new FullShooterParams(13.486066, 31.663028, 1.554488));
        SHUTTLE_MAP.put(15.0000, new FullShooterParams(13.848600, 30.675814, 1.558758));
        SHUTTLE_MAP.put(15.5000, new FullShooterParams(14.216455, 29.731555, 1.563155));
        SHUTTLE_MAP.put(16.0000, new FullShooterParams(14.589381, 28.827790, 1.567677));
        SHUTTLE_MAP.put(16.5000, new FullShooterParams(14.967143, 27.962192, 1.572322));
        SHUTTLE_MAP.put(17.0000, new FullShooterParams(15.349524, 27.132568, 1.577089));
        SHUTTLE_MAP.put(17.5000, new FullShooterParams(15.736325, 26.336854, 1.581977));
        SHUTTLE_MAP.put(18.0000, new FullShooterParams(16.127357, 25.573109, 1.586983));
        SHUTTLE_MAP.put(18.5000, new FullShooterParams(16.522449, 24.839511, 1.592106));
        SHUTTLE_MAP.put(19.0000, new FullShooterParams(16.921441, 24.134350, 1.597345));
        SHUTTLE_MAP.put(19.5000, new FullShooterParams(17.324186, 23.456020, 1.602698));
        SHUTTLE_MAP.put(20.0000, new FullShooterParams(17.730547, 22.803017, 1.608162));
    };

    public static final class SotfConstants {
        public static final double defaultLoopDtSeconds = 0.02;
        public static final double minLoopDtSeconds = 1.0e-4;
        public static final double maxLoopDtSeconds = 0.1;

        public static final double latencyCompensationSeconds = 0.30;
        public static final double minDistanceMeters = 1.0e-6;
        public static final double minDerivativeDistanceDeltaMeters = 1.0e-9;

        public static final int maxRecursiveIterations = 5;

        public static final int maxNewtonIterations = 15;
        public static final double newtonToleranceSeconds = 0.001;
        public static final double newtonMinDerivativeMagnitude = 0.01;
        public static final double tofDerivativeStepMeters = 0.01;

        public static final double maxValidDistanceMeters = 10.0;
        public static final double pointAtTargetToleranceDeg = 5.0;

        // Shooter physical offset from turret center, meters.
        public static final double turretShooterOffsetX = 0.0;
        public static final double turretShooterOffsetY = 0.0;

        private SotfConstants() {
        }
    }
}
