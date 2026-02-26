package frc.robot.subsystems.dyerotor;

public class DyeRotorConstants {
    // Can Id
    public static final int rotorCanId = 29;
    public static final int feederCanId = 39;

    // Motor direction
    public static final boolean rotorIsInverted = false;
    public static final boolean feederIsInverted = false;

    // Current
    public static final int rotorMotorCurrent = 40;
    public static final int feederMotorCurrent = 40;

    // Encoder stuff
    public static final double rotorEncoderPositionFactor = 1;
    public static final double rotorEncoderVelocityFactor = 1;
    public static final double feederEncoderPositionFactor = 1;
    public static final double feederEncoderVelocityFactor = 1;

    // PID stiff
    public static final double rotorKp = 0;
    public static final double rotorKi = 0;
    public static final double rotorKd = 0;

    public static final double feederKp = 0;
    public static final double feederKi = 0;
    public static final double feederKd = 0;

    public static final double rotorRunSpeed = 1;
    public static final double feederRunSpeed = 1;

    // no clue what these should be tbh
    public static final double rotorMotorReduction = 7 / 1;
    public static final double rollerMotorReduction = 7 / 1;
}
