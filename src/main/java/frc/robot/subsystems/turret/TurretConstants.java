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
        SHOOTER_MAP.put(1.0, new FullShooterParams(5, 80, 3));
        SHOOTER_MAP.put(3.0, new FullShooterParams(9, 75, 3));
        SHOOTER_MAP.put(5.0, new FullShooterParams(10, 65, 3));
        SHOOTER_MAP.put(7.0, new FullShooterParams(11, 60, 3));
    };

    public static final double[][] turretRPMData = {
        {1, 5},
        {3, 9},
        {5, 10},
        {7, 11}
    };

    public static final double[][] turretHoodData = {
        {1, 80},
        {3, 75},
        {5, 65},
        {7, 60}
    };
}
