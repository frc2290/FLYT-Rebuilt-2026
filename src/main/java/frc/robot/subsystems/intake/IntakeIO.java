package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        // Roller position in meters.
        public double drivePositionMeters = 0.0;
        // Roller surface speed in m/s.
        public double driveSpeed = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public double deployPosition = 0;
        public double deployVelocityRadPerSec = 0.0;
        public double deployAppliedVolts = 0.0;
        public double deployCurrentAmps = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    
    public default void setIntakeSpeed(double speed) {}
    public default void setIntakeVoltage(double volts) {}
    public default void setDeployPosition(double angle, boolean useProfile) {}
    public default boolean deployAtSetpoint() { return false; }
    public default boolean rollerAtSetpoint() { return false; }
}
