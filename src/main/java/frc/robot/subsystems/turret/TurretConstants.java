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

    public static final InterpolatingTreeMap<Double, FullShooterParams> SHOOTER_MAP = new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(), ShootOnTheFly::interpolateParams);
    static {
        SHOOTER_MAP.put(1.0, new FullShooterParams(4.754379067,66.53955795,0.528319644));
        SHOOTER_MAP.put(1.1, new FullShooterParams(4.830787878,65.18378018,0.542533283));
        SHOOTER_MAP.put(1.2, new FullShooterParams(4.908917603,63.96375201,0.556917241));
        SHOOTER_MAP.put(1.3, new FullShooterParams(4.988271318,62.86387746,0.571383171));
        SHOOTER_MAP.put(1.4, new FullShooterParams(5.068447942,61.87004674,0.585863226));
        SHOOTER_MAP.put(1.5, new FullShooterParams(5.149125483,60.96970673,0.600305841));
        SHOOTER_MAP.put(1.6, new FullShooterParams(5.230046509,60.15180835,0.61467223));
        SHOOTER_MAP.put(1.7, new FullShooterParams(5.311005925,59.40669064,0.628933578));
        SHOOTER_MAP.put(1.8, new FullShooterParams(5.39184089,58.72593836,0.643068799));
        SHOOTER_MAP.put(1.9, new FullShooterParams(5.472422603,58.1022346,0.657062786));
        SHOOTER_MAP.put(2.0, new FullShooterParams(5.55264966,57.52921988,0.670905038));
        SHOOTER_MAP.put(2.1, new FullShooterParams(5.632442708,57.00136345,0.684588598));
        SHOOTER_MAP.put(2.2, new FullShooterParams(5.711740143,56.51384894,0.698109233));
        SHOOTER_MAP.put(2.3, new FullShooterParams(5.790494668,56.0624744,0.711464785));
        SHOOTER_MAP.put(2.4, new FullShooterParams(5.86867052,55.64356595,0.724654684));
        SHOOTER_MAP.put(2.5, new FullShooterParams(5.946241242,55.25390368,0.737679562));
        SHOOTER_MAP.put(2.6, new FullShooterParams(6.023187893,54.89065837,0.75054095));
        SHOOTER_MAP.put(2.7, new FullShooterParams(6.099497599,54.55133748,0.763241054));
        SHOOTER_MAP.put(2.8, new FullShooterParams(6.175162384,54.23373924,0.775782568));
        SHOOTER_MAP.put(2.9, new FullShooterParams(6.250178221,53.93591347,0.788168539));
        SHOOTER_MAP.put(3.0, new FullShooterParams(6.324544267,53.65612825,0.800402258));
        SHOOTER_MAP.put(3.1, new FullShooterParams(6.398262231,53.39284141,0.812487174));
        SHOOTER_MAP.put(3.2, new FullShooterParams(6.471335868,53.14467624,0.82442683));
        SHOOTER_MAP.put(3.3, new FullShooterParams(6.543770561,52.91040057,0.836224814));
        SHOOTER_MAP.put(3.4, new FullShooterParams(6.615572981,52.68890896,0.847884718));
        SHOOTER_MAP.put(3.5, new FullShooterParams(6.686750809,52.47920723,0.859410109));
        SHOOTER_MAP.put(3.6, new FullShooterParams(6.757312503,52.28039921,0.870804512));
        SHOOTER_MAP.put(3.7, new FullShooterParams(6.827267117,52.09167527,0.882071384));
        SHOOTER_MAP.put(3.8, new FullShooterParams(6.896624144,51.91230234,0.893214112));
        SHOOTER_MAP.put(3.9, new FullShooterParams(6.965393389,51.74161532,0.904235996));
        SHOOTER_MAP.put(4.0, new FullShooterParams(7.033584867,51.57900949,0.915140248));
        SHOOTER_MAP.put(4.1, new FullShooterParams(7.10120872,51.423934,0.925929989));
        SHOOTER_MAP.put(4.2, new FullShooterParams(7.168275144,51.27588606,0.936608242));
        SHOOTER_MAP.put(4.3, new FullShooterParams(7.234794338,51.13440597,0.947177936));
        SHOOTER_MAP.put(4.4, new FullShooterParams(7.300776453,50.99907261,0.957641904));
        SHOOTER_MAP.put(4.5, new FullShooterParams(7.36623156,50.86949959,0.968002885));
        SHOOTER_MAP.put(4.6, new FullShooterParams(7.431169618,50.74533179,0.978263524));
        SHOOTER_MAP.put(4.7, new FullShooterParams(7.495600447,50.62624227,0.988426378));
        SHOOTER_MAP.put(4.8, new FullShooterParams(7.559533719,50.51192961,0.998493911));
        SHOOTER_MAP.put(4.9, new FullShooterParams(7.622978933,50.40211548,1.008468506));
        SHOOTER_MAP.put(5.0, new FullShooterParams(7.685945411,50.29654251,1.018352459));
        SHOOTER_MAP.put(5.1, new FullShooterParams(7.748442289,50.19497236,1.028147987));
        SHOOTER_MAP.put(5.2, new FullShooterParams(7.810478509,50.09718403,1.03785723));
        SHOOTER_MAP.put(5.3, new FullShooterParams(7.872062817,50.00297233,1.047482252));
        SHOOTER_MAP.put(5.4, new FullShooterParams(7.933203762,49.91214647,1.057025045));
        SHOOTER_MAP.put(5.5, new FullShooterParams(7.993909691,49.8245289,1.066487533));
        SHOOTER_MAP.put(5.6, new FullShooterParams(8.054188758,49.73995412,1.075871572));
        SHOOTER_MAP.put(5.7, new FullShooterParams(8.114048915,49.65826773,1.085178955));
        SHOOTER_MAP.put(5.8, new FullShooterParams(8.173497922,49.57932553,1.094411414));
        SHOOTER_MAP.put(5.9, new FullShooterParams(8.232543344,49.50299264,1.103570621));
        SHOOTER_MAP.put(6.0, new FullShooterParams(8.29119256,49.42914283,1.112658193));
        SHOOTER_MAP.put(6.1, new FullShooterParams(8.349452761,49.35765783,1.121675691));
        SHOOTER_MAP.put(6.2, new FullShooterParams(8.407330958,49.28842667,1.130624625));
        SHOOTER_MAP.put(6.3, new FullShooterParams(8.46483398,49.22134519,1.139506457));
        SHOOTER_MAP.put(6.4, new FullShooterParams(8.521968487,49.15631551,1.148322598));
        SHOOTER_MAP.put(6.5, new FullShooterParams(8.578740967,49.09324554,1.157074415));
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
