package frc.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ShootOnTheFly {
    public class SOTFResult {
        double yaw;   // turretAngle
        double pitch; // hoodAngle
        double vel;   // exitVelocity
        
        public SOTFResult(double yaw, double pitch, double vel) {
            this.yaw = yaw;
            this.pitch = pitch;
            this.vel = vel;
        }
    }

    public SOTFResult calculate(Translation2d goalLocation, Pose2d robotPose, ChassisSpeeds robotSpeed) {
        double latency = 0.15; // Tuned constant
        Translation2d futurePos = robotPose.getTranslation().plus(
            new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
        );

        // 2. GET TARGET VECTOR
        Translation2d targetVec = goalLocation.minus(futurePos);
        double dist = targetVec.getNorm();

        // 3. CALCULATE IDEAL SHOT (Stationary)
        // Note: This returns HORIZONTAL velocity component
        double idealHorizontalSpeed = 2;//ShooterTable.getSpeed(dist);

        // 4. VECTOR SUBTRACTION
        Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
        Translation2d shotVec = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

        // 5. CONVERT TO CONTROLS
        double turretAngle = shotVec.getAngle().getDegrees();
        double newHorizontalSpeed = shotVec.getNorm();

        // 6. SOLVE FOR NEW PITCH/RPM
        // Assuming constant total exit velocity, variable hood:
        double totalExitVelocity = 15.0; // m/s
        // Clamp to avoid domain errors if we need more speed than possible
        double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
        double newPitch = Math.acos(ratio);

        return new SOTFResult(turretAngle, Math.toDegrees(newPitch), totalExitVelocity);
    }
}
