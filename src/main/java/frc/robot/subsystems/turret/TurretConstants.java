package frc.robot.subsystems.turret;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.utils.ShootOnTheFly;
import frc.utils.ShootOnTheFly.FullShooterParams;

public class TurretConstants {
    public static final double turretHeight = inchesToMeters(24);

    public static final int turretTurnMotor = 50;
    public static final double turretTurnReduction = 7 / 1;
    public static final int turretShootMotor = 51;
    public static final double turretShootReduction = 7 / 1;
    public static final int turretHoodMotor = 52;
    public static final double turretHoodReduction = 90 / 1;

    public static final int turretCurrentLimit = 50;

    public static final double turretTurnSimP = 1.0;
    public static final double turretTurnSimI = 0.0;
    public static final double turretTurnSimD = 0.0;

    public static final double turretShootSimP = 1.0;
    public static final double turretShootSimI = 0.0;
    public static final double turretShootSimD = 0.0;

    public static final double turretHoodSimP = 1.0;
    public static final double turretHoodSimI = 0.0;
    public static final double turretHoodSimD = 0.0;

    // Device CAN ID(s)
    public static final int turretCanId = 20;
    public static final int hoodCanId = 21;
    public static final int flywheel1CanId = 22;
    public static final int flywheel2CanId = 23;

    // Turret config
    public static final boolean turretIsInverted = false;
    public static final int turretMotorCurrent = 40;
    public static final double turretEncoderPositionFactor = 1;
    public static final double turretEncoderVelocityFactor = 1;
    public static final double turretKp = 0.001;
    public static final double turretKi = 0.0;
    public static final double turretKd = 0.0;

    // Hood config
    public static final boolean hoodIsInverted = false;
    public static final int hoodMotorCurrent = 40;
    public static final boolean hoodEncoderInverted = false;
    public static final double hoodEncoderPositionFactor = 1;
    public static final double hoodEncoderVelocityFactor = 1;
    public static final double hoodKp = 0.001;
    public static final double hoodKi = 0.0;
    public static final double hoodKd = 0.0;

    // Flywheel config
    public static final boolean flywheelIsInverted = false;
    public static final int flywheelMotorCurrent = 40;
    public static final double flywheelEncoderPositionFactor = 1;
    public static final double flywheelEncoderVelocityFactor = 1;
    public static final double flywheelKp = 0.001;
    public static final double flywheelKi = 0.0;
    public static final double flywheelKd = 0.0;


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
}
