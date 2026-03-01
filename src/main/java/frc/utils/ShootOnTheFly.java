package frc.utils;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootOnTheFly {
    public static ShootOnTheFly instance = null;
    private LinearInterpolator shootAngleInterp = new LinearInterpolator();
    private LinearInterpolator shootSpeedInterp = new LinearInterpolator();

    public record FullShooterParams(double speedMetersPerSecond, double hoodAngle, double timeOfFlight) {
    }

    private InterpolatingTreeMap<Double, FullShooterParams> SHOOTER_MAP;

    public static FullShooterParams interpolateParams(FullShooterParams startValue, FullShooterParams endValue,
            double t) {
        return new FullShooterParams(
                MathUtil.interpolate(startValue.speedMetersPerSecond(), endValue.speedMetersPerSecond(), t),
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

    public SOTFResult calculateRecursiveTOF(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeeds) {
        Translation2d robotVelocity = new Translation2d(robotSpeeds.vxMetersPerSecond, robotSpeeds.vyMetersPerSecond);

        double latencyCompensation = 0.0;
        Translation2d futurePos = robotPose.getTranslation().plus(
                robotVelocity.times(latencyCompensation));

        // robot-relative goal position
        Translation2d toGoal = goalLocation.minus(futurePos);
        // the calculated final goal position relative to the robot
        Translation2d ballGoal = toGoal;

        FullShooterParams params = null;
        double distance = ballGoal.getNorm();
        double k = 0.5; // linear drag constant, tune with testing, K can not equal 0!!

        // the reason this is done is because when you want to apply the velocity
        // corrections, you also need to know how long the ball will be in the air for.
        // this is important because that allows us to take the velocity of the robot
        // and convert it into an actual position we can use to point towards and get
        // the distance of for the LUT
        for (int i = 0; i < 5; i++) {
            params = SHOOTER_MAP.get(distance);
            double tof = params.timeOfFlight();
            double alphaTof = (1.0 - Math.exp(-k * tof)) / k;
            // the reason the TOF is negative isn't for the TOF, but rather the velocity.
            // here, instead of wanting field-relative robot velocity, we want the
            // robot-relative goal velocity, which will be the opposite.
            ballGoal = toGoal.plus(robotVelocity.times(-alphaTof));
            distance = ballGoal.getNorm();
        }

        // we get the params one more time with the updated distance...
        params = SHOOTER_MAP.get(distance);
        // ...and then return the params and direction
        return new SOTFResult(ballGoal.getAngle().getDegrees(), params.hoodAngle(), params.speedMetersPerSecond());
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
        double baselineVelocity = distance / baseline.timeOfFlight();

        // 4. Build target velocity vector
        Translation2d targetVelocity = targetDirection.times(baselineVelocity);

        // 5. THE MAGIC: subtract robot velocity
        Translation2d shotVelocity = targetVelocity.minus(robotVelocity);

        // 6. Extract results
        Rotation2d turretAngle = shotVelocity.getAngle();
        double requiredVelocity = shotVelocity.getNorm();

        // Both Calculation
        // FullShooterParams baseline = SHOOTER_MAP.get(distance);
        // double baselineVelocity = distance / baseline.timeOfFlight;
        double velocityRatio = requiredVelocity / baselineVelocity;

        // Split the correction: sqrt gives equal "contribution" from each
        double speedFactor = Math.sqrt(velocityRatio);
        double hoodFactor = Math.sqrt(velocityRatio);

        // Apply shooter speed scaling (m/s)
        double adjustedSpeedMetersPerSecond = baseline.speedMetersPerSecond() * speedFactor;

        // Apply hood adjustment (changes horizontal component)
        double totalVelocity = baselineVelocity / Math.cos(Math.toRadians(baseline.hoodAngle()));
        double targetHorizFromHood = baselineVelocity * hoodFactor;
        double ratio = MathUtil.clamp(targetHorizFromHood / totalVelocity, 0.0, 1.0);
        double adjustedHood = Math.toDegrees(Math.acos(ratio));

        return new SOTFResult(turretAngle.getDegrees(), adjustedHood, adjustedSpeedMetersPerSecond);
        // return new ShooterCommand(adjustedSpeedMetersPerSecond, adjustedHood);
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
