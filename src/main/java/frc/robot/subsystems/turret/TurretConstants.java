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
    public static final int turretMotorCurrent = 40;
    public static final int hoodMotorCurrent = 40;
    public static final int flywheelMotorCurrent = 40;
    public static final int turretCurrentLimit = 50;

    // --- Physical Dimensions & Mechanics ---
    public static final double turretHeight = inchesToMeters(24);
    public static final double shooterWheelDiameterMeters = 3.0 * 0.0254;
    public static final double rangeTurret = 2.0;
    public static final double numTeethTurret = 240.0;
    public static final double numTeethPulley1 = 22.0;
    public static final double numTeethPulley2 = 23.0;
    public static final double numTeethMotor = 11.0;
    public static final double turretTurnReduction = 7 / 1;
    public static final double turretShootReduction = 24.0 / 35.0;
    public static final double hoodMotorToEncoderReduction = (23.0 / 1.0) * (32.0 / 15.0);
    public static final double hoodEncoderToHoodReduction = 335.0 / 45.0;
    public static final double turretHoodReduction = hoodMotorToEncoderReduction * hoodEncoderToHoodReduction;

    // --- Encoder & Conversion Factors ---
    public static final boolean hoodEncoderInverted = false;
    public static final double encoderOffset = 0;
    public static final double hoodEncoderZeroOffset = 0.65;
    public static final double hoodAngleOffset = 3.87;
    public static final double hoodShotAngleOffset = 74.8;
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
    public static final double turretKp = 0.001;
    public static final double turretKi = 0.0;
    public static final double turretKd = 0.0;
    public static final DCMotor turretGearbox = DCMotor.getNeoVortex(1);
    public static final double turretFreeSpeedRPM =
            Units.radiansPerSecondToRotationsPerMinute(turretGearbox.freeSpeedRadPerSec);
    public static final double turretTheoreticalKv =
            12.0 / (turretFreeSpeedRPM * turretEncoderVelocityFactor);

    public static final double hoodKp = 0.05;
    public static final double hoodKi = 0.0;
    public static final double hoodKd = 0.0;
    public static final double hoodff = 0;
    public static final DCMotor hoodGearbox = DCMotor.getNEO(1);
    public static final double hoodFreeSpeedRPM =
            Units.radiansPerSecondToRotationsPerMinute(hoodGearbox.freeSpeedRadPerSec);
    public static final double hoodKv = 1 / ((hoodFreeSpeedRPM / hoodMotorToEncoderReduction) * hoodEncoderVelocityFactor);

    public static final double flywheelKp = 0.67;
    public static final double flywheelKi = 0.0;
    public static final double flywheelKd = 0.0;//0.28
    public static final double shooterff = 0;
    public static final DCMotor flywheelGearbox = DCMotor.getNeoVortex(1);
    public static final double flywheelFreeSpeedRPM =
            Units.radiansPerSecondToRotationsPerMinute(flywheelGearbox.freeSpeedRadPerSec);
    public static final double flywheelKv = (1.27 / (flywheelFreeSpeedRPM * flywheelEncoderVelocityFactor));

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
    public static final double[][] turretRPMData = {
            { 1.5, 6.5 },
            { 3, 7 },
            { 5, 8 },
    };

    public static final double[][] turretHoodData = {
            { 1.5, 80 },
            { 3, 65 },
            { 5, 50 },
    };

    public static final InterpolatingTreeMap<Double, FullShooterParams> SHOOTER_MAP = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), ShootOnTheFly::interpolateParams);
    static {
        SHOOTER_MAP.put(1.0000, new FullShooterParams(5.549333, 80.947728, 0.907106));
        SHOOTER_MAP.put(1.2500, new FullShooterParams(5.539373, 76.043441, 0.879143));
        SHOOTER_MAP.put(1.5000, new FullShooterParams(5.663275, 71.678186, 0.879233));
        SHOOTER_MAP.put(1.7500, new FullShooterParams(5.811649, 67.734279, 0.879810));
        SHOOTER_MAP.put(2.0000, new FullShooterParams(5.943120, 65.134988, 0.883561));
        SHOOTER_MAP.put(2.2500, new FullShooterParams(6.088001, 63.163706, 0.894343));
        SHOOTER_MAP.put(2.5000, new FullShooterParams(6.241991, 61.455738, 0.907915));
        SHOOTER_MAP.put(2.7500, new FullShooterParams(6.400864, 59.962799, 0.923212));
        SHOOTER_MAP.put(3.0000, new FullShooterParams(6.561141, 58.617382, 0.938912));
        SHOOTER_MAP.put(3.2500, new FullShooterParams(6.983379, 65.765750, 1.129472));
        SHOOTER_MAP.put(3.5000, new FullShooterParams(7.092456, 62.601435, 1.112385));
        SHOOTER_MAP.put(3.7500, new FullShooterParams(7.046954, 55.522359, 0.992186));
        SHOOTER_MAP.put(4.0000, new FullShooterParams(7.202634, 54.481024, 1.005608));
        SHOOTER_MAP.put(4.2500, new FullShooterParams(7.363941, 53.906269, 1.027625));
        SHOOTER_MAP.put(4.5000, new FullShooterParams(7.515656, 52.907363, 1.038667));
        SHOOTER_MAP.put(4.7500, new FullShooterParams(7.673770, 52.565551, 1.062916));
        SHOOTER_MAP.put(5.0000, new FullShooterParams(7.823549, 51.764978, 1.075609));
        SHOOTER_MAP.put(5.2500, new FullShooterParams(7.974482, 51.295725, 1.094566));
        SHOOTER_MAP.put(5.5000, new FullShooterParams(8.123487, 50.909204, 1.114349));
        SHOOTER_MAP.put(5.7500, new FullShooterParams(8.268206, 50.192344, 1.125551));
        SHOOTER_MAP.put(6.0000, new FullShooterParams(8.412179, 49.561814, 1.137753));
        SHOOTER_MAP.put(6.2500, new FullShooterParams(8.555571, 49.574790, 1.164044));
        SHOOTER_MAP.put(6.5000, new FullShooterParams(8.695834, 49.180838, 1.180105));
        SHOOTER_MAP.put(6.7500, new FullShooterParams(8.834536, 48.554559, 1.189882));
        SHOOTER_MAP.put(7.0000, new FullShooterParams(8.973582, 47.758843, 1.194848));
    };
}
