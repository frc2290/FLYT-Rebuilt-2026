package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double driveSpeed = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;

        public Rotation2d deployPosition = Rotation2d.kZero;
        public double deployVelocityRadPerSec = 0.0;
        public double deployAppliedVolts = 0.0;
        public double deployCurrentAmps = 0.0;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
    
    public default void setIntakeSpeed(double speed) {}
    public default void setDeployPosition(Rotation2d rotation) {}
}
