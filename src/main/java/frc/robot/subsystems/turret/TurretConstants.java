package frc.robot.subsystems.turret;

import static edu.wpi.first.math.util.Units.inchesToMeters;

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

    public static final double[][] turretRPMData = {
        {1, 500},
        {2, 1000},
        {3, 1500}
    };

    public static final double[][] turretHoodData = {
        {1, 70},
        {2, 50},
        {3, 40}
    };
}
