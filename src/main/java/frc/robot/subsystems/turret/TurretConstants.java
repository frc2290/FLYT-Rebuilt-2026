package frc.robot.subsystems.turret;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import frc.utils.ShootOnTheFly;
import frc.utils.ShootOnTheFly.FullShooterParams;

public class TurretConstants {
    public static final double turretHeight = inchesToMeters(24);

    public static final int turretTurnMotor = 50;
    public static final double turretTurnReduction = 7/1;
    public static final int turretShootMotor = 51;
    public static final double turretShootReduction = 7/1;
    public static final int turretHoodMotor = 52;
    public static final double turretHoodReduction = 90/1;

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

    public static final InterpolatingTreeMap<Double, FullShooterParams> SHOOTER_MAP = new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), ShootOnTheFly::interpolateParams);
    static {
        SHOOTER_MAP.put(1.5, new FullShooterParams(6.5, 80, 1.25));
        SHOOTER_MAP.put(3.0, new FullShooterParams(7, 65, 1.1));
        SHOOTER_MAP.put(5.0, new FullShooterParams(8, 50, 1.2));
    };

    public static final double[][] turretRPMData = {
        {1.5, 6.5},
        {3, 7},
        {5, 8},
    };

    public static final double[][] turretHoodData = {
        {1.5, 80},
        {3, 65},
        {5, 50},
    };
}
