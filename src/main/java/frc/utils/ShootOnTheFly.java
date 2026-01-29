package frc.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class ShootOnTheFly {
    public static ShootOnTheFly instance = null;
    private LinearInterpolator shootAngleInterp = new LinearInterpolator();
    private LinearInterpolator shootSpeedInterp = new LinearInterpolator();

    public record FullShooterParams(double rpm, double hoodAngle, double timeOfFlight) {
    }

    private InterpolatingTreeMap<Double, FullShooterParams> SHOOTER_MAP;

    public static FullShooterParams interpolateParams(FullShooterParams startValue, FullShooterParams endValue,
            double t) {
        return new FullShooterParams(
                MathUtil.interpolate(startValue.rpm(), endValue.rpm(), t),
                MathUtil.interpolate(startValue.hoodAngle(), endValue.hoodAngle(), t),
                MathUtil.interpolate(startValue.timeOfFlight(), endValue.timeOfFlight(), t));
    }

    public static class SOTFResult {
        public double yaw; // turretAngle
        public double pitch; // hoodAngle
        public double vel; // exitVelocity

        public SOTFResult(double yaw, double pitch, double vel) {
            this.yaw = yaw;
            this.pitch = pitch;
            this.vel = vel;
        }
    }

    public void addShootInterpData(InterpolatingTreeMap<Double, FullShooterParams> shooterMap) {
        SHOOTER_MAP = shooterMap;
    }

    public void addShootAngleInterpData(double[][] data) {
        shootAngleInterp.build_table(data);
    }

    public double getShootAngleInterp(double dist) {
        return shootAngleInterp.getInterpolatedValue(dist);
    }

    public void addShootSpeedInterpData(double[][] data) {
        shootSpeedInterp.build_table(data);
    }

    public double getShootSpeedInterp(double dist) {
        return shootSpeedInterp.getInterpolatedValue(dist);
    }

    public SOTFResult calculateTOF(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        Translation2d robotVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);
        double latencyCompensation = 0.15;

        // 1. Project future position
        Translation2d futurePos = robotPose.getTranslation().plus(
                robotVelocity.times(latencyCompensation));

        // 2. Get target vector
        Translation2d toGoal = goalLocation.minus(futurePos);
        double distance = toGoal.getNorm();
        Translation2d targetDirection = toGoal.div(distance);

        // 3. Look up baseline velocity from table
        FullShooterParams baseline = SHOOTER_MAP.get(distance);
        double baselineVelocity = distance / baseline.timeOfFlight;

        // 4. Build target velocity vector
        Translation2d targetVelocity = targetDirection.times(baselineVelocity);

        // 5. THE MAGIC: subtract robot velocity
        Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

        // 6. Extract results
        Rotation2d turretAngle = shotVelocity.getAngle();
        double requiredVelocity = shotVelocity.getNorm();

        // Both Calculation
        //FullShooterParams baseline = SHOOTER_MAP.get(distance);
        //double baselineVelocity = distance / baseline.timeOfFlight;
        double velocityRatio = requiredVelocity / baselineVelocity;

        // Split the correction: sqrt gives equal "contribution" from each
        double rpmFactor = Math.sqrt(velocityRatio);
        double hoodFactor = Math.sqrt(velocityRatio);

        // Apply RPM scaling
        double adjustedRpm = baseline.rpm * rpmFactor;

        // Apply hood adjustment (changes horizontal component)
        double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.hoodAngle));
        double targetHorizFromHood = baselineVelocity * hoodFactor;
        double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
        double adjustedHood = Math.toDegrees(Math.acos(ratio));

        return new SOTFResult(turretAngle.getDegrees(), adjustedHood, adjustedRpm);
        // return new ShooterCommand(adjustedRpm, adjustedHood);
    }

    public SOTFResult calculate(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeed) {
        double latency = 0.15; // Tuned constant
        Translation2d futurePos = robotPose.getTranslation().plus(
                new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency));

        // 2. GET TARGET VECTOR
        Translation2d targetVec = goalLocation.minus(futurePos);
        double dist = targetVec.getNorm();
        Logger.recordOutput("SOTF Dist", dist);

        // 3. CALCULATE IDEAL SHOT (Stationary)
        // Note: This returns HORIZONTAL velocity component
        // idealSpeed_Horizontal = Total_Speed * cos(release_angle);
        double interpSpeed = shootSpeedInterp.getInterpolatedValue(dist);
        double interpAngle = shootAngleInterp.getInterpolatedValue(dist);
        Logger.recordOutput("SOTF Speed", interpSpeed);
        Logger.recordOutput("SOTF Angle", interpAngle);
        double idealHorizontalSpeed = interpSpeed * Math.cos(Math.toRadians(interpAngle));
        Logger.recordOutput("SOTF HorzSpeed", idealHorizontalSpeed);

        // 4. VECTOR SUBTRACTION
        Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        Translation2d shotVec = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

        // 5. CONVERT TO CONTROLS
        double turretAngle = shotVec.getAngle().getDegrees();
        double newHorizontalSpeed = shotVec.getNorm();

        // 6. SOLVE FOR NEW PITCH/RPM
        // Assuming constant total exit velocity, variable hood:
        double totalExitVelocity = 8.0; // m/s
        // Clamp to avoid domain errors if we need more speed than possible
        double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
        double newPitch = Math.acos(ratio);

        return new SOTFResult(turretAngle, Math.toDegrees(newPitch), totalExitVelocity);
    }

    public static ShootOnTheFly getInstance() {
        if (instance == null) {
            instance = new ShootOnTheFly();
        }
        return instance;
    }

    private ShootOnTheFly() {
    }
}
