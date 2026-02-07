package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DyeRotorIO {

    @AutoLog
    public static class DyeRotorIOInputs {
        public double dyeRotorVel = 0;

    }


    public default void updateInputs(DyeRotorIOInputs inputs) {}

    //turrent control functions
    public default void setTurnPosition(Rotation2d rotation) {}
    public default void shootFuel() {}
    public default void setHoodAngle(double angle) {}
    public default void setShooterSpeed(double speed) {}
}