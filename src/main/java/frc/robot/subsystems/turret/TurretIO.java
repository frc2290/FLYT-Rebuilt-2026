package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double turretAngle = 0;
        public double turretSpeed = 0;
        public double turretHoodAngle = 0;
        public double turretAngleSetpoint = 0;

        public boolean turretConnected = false;
        public double turretPositionRad = 0.0;
        public double turretVelocityRadPerSec = 0.0;
        public double turretAppliedVolts = 0.0;
        public double turretCurrentAmps = 0.0;

        public boolean hoodConnected = false;
        public double hoodPositionDeg = 0.0;
        public double hoodVelocityDegPerSec = 0.0;
        public double hoodAppliedVolts = 0.0;
        public double hoodCurrentAmps = 0.0;

        public boolean flywheelConnected = false;
        public double flywheelPositionRad = 0.0;
        public double flywheelVelocity = 0.0;
        public double flywheelAppliedVolts = 0.0;
        public double flywheelCurrentAmps = 0.0;
    }

    public default void updateInputs(TurretIOInputs inputs) {};

    public default void setTurnPosition(Rotation2d rotation) {};
    public default void shootFuel() {};
    public default void setHoodAngle(double angle) {};
    public default void setShooterSpeed(double speed) {};
    public default void setShotAngle(double angle) {};
}
