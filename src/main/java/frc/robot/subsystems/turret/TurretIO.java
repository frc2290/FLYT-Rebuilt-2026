package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double turretAngle = 0;
        public double hoodAngle = 0;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setTurnPosition(Rotation2d rotation) {};
    public default void shootFuel() {};  
}
