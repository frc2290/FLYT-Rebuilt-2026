package frc.robot.subsystems.dyerotor;

import org.littletonrobotics.junction.AutoLog;

public interface DyeRotorIO {
    @AutoLog
    public static class DyeRotorIOInputs {
        public double rotorSpeed = 0;
        public double rotorAppliedVolts = 0.0;
        public double rotorCurrentAmps = 0.0;
        
        public double feederSpeed = 0;
        public double feederAppliedVolts = 0.0;
        public double feederCurrentAmps = 0.0;
    }

    public default void updateInputs(DyeRotorIOInputs inputs) {}

    public default void setRotorSpeed(double speed) {}
    public default void setFeederSpeed(double speed) {}
}