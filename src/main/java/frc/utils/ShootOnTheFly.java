package frc.utils;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

public class ShootOnTheFly {
    public static ShootOnTheFly instance = null;
    private LinearInterpolator shootAngleInterp = new LinearInterpolator();
    private LinearInterpolator shootSpeedInterp = new LinearInterpolator();

    public static class SOTFResult {
        public double yaw;   // turretAngle
        public double pitch; // hoodAngle
        public double vel;   // exitVelocity
        
        public SOTFResult(double yaw, double pitch, double vel) {
            this.yaw = yaw;
            this.pitch = pitch;
            this.vel = vel;
        }
    }

    public void addShootAngleInterpData(double[][] data) {
        shootAngleInterp.build_table(data);
    }

    public void addShootSpeedInterpData(double[][] data) {
        shootSpeedInterp.build_table(data); 
    }

    public SOTFResult calculate(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeed) {
        double latency = 0.15; // Tuned constant
        Translation2d futurePos = robotPose.getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
        );

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

    private ShootOnTheFly() {}
}
