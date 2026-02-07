package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public double turretAngle = 0;
        public double turretSpeed = 0;
        public double turretHoodAngle = 0;
        public double turretAngleSetpoint = 0;
    }


    public default void updateInputs(ClimbIOInputs inputs) {}

    //turrent control functions
    public default void setClimbPos(double position) {}
}
