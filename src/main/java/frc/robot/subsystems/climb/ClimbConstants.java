package frc.robot.subsystems.climb;

public class ClimbConstants {
    // --- CAN IDs ---
    public static final int climbCanId = 10; 

    // --- Motor Configurations ---
    public static final boolean climbIsInverted = false; 
    public static final int climbMotorCurrent = 40; 

    // --- Encoder & Conversion Factors ---
    public static final double climbEncoderPositionFactor = 10; 
    public static final double climbEncoderVelocityFactor = 10; 

    // --- Control Loop Constants (PID) ---
    public static final double climbKp = 10; 
    public static final double climbKi = 10; 
    public static final double climbKd = 10; 
}
