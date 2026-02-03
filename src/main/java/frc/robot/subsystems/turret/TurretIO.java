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
    }


    public default void updateInputs(TurretIOInputs inputs) {}

    //turrent control functions
    public default void setTurnPosition(Rotation2d rotation) {}
    public default void shootFuel() {}
    public default void setHoodAngle(double angle) {}
    public default void setShooterSpeed(double speed) {}
}
