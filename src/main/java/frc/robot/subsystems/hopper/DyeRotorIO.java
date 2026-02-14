package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DyeRotorIO {

    @AutoLog
    public static class DyeRotorIOInputs {
        public double dyeRotorVel = 0;
        public double feedRate = 0;
        public boolean isRotorRunning = false;

    }


    public default void updateInputs(DyeRotorIOInputs inputs) {}

    //turrent control functions
    public default void runDyeRotor(boolean run) {}
    public default void setDyeRotorSpeed(double speed) {}
    public default void setFeedRate(double feedRate) {}

}