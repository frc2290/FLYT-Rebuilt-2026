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


        // --- Dye Rotor Gear Ratios ---
    // Motor RPM = Mechanism RPM * Gear Ratio
    public static final double rotorGearRatio = 3.0*4.0*(72.0/16.0);
    public static final double feedGearRatio = 60.0 / 37.0;

    // Encoder stuff
    public static final double rotorEncoderPositionFactor = 1.0 /rotorGearRatio;
    public static final double rotorEncoderVelocityFactor = 1.0 /rotorGearRatio;
    public static final double feederEncoderPositionFactor = 1.0/feedGearRatio;
    public static final double feederEncoderVelocityFactor = 1.0/feedGearRatio;

    // PID stiff
    public static final double rotorKp = 0;
    public static final double rotorKi = 0;
    public static final double rotorKd = 0;

    public static final double feederKp = 0;
    public static final double feederKi = 0;
    public static final double feederKd = 0;

    // Throughput target used by runDyeRotor(true)
    public static final double defaultTargetBps = 10;

    // --- Physical Dimensions ---
    public static final double ballsPerRotation = 8.0;
    public static final double fuelDiameterInches = 150/25.4; // 150mm in inches
    public static final double feedWheelRadiusInches = 1.625/2.0; // 1.625in diameter
    public static final double overfeedRatio = 1.18; //Rate balls are fed realitive to the rotor speed. Based on Wildstang Calcs

// --- Feedforward (kFF) Calculations ---
// Free speeds of the two motors
public static final double vortexFreeSpeedRpm = 6784.0;
public static final double neoFreeSpeedRpm = 5676.0;

// kV converts a target RPM into a 0.0 to 12.0 motor voltage command.
// Formula: 12 volts / (MotorFreeSpeed / GearRatio)
// This ensures that commanding the theoretical max mechanism RPM outputs exactly 12.0 (Max Voltage).
public static final double rotorKv = 12.0 / (vortexFreeSpeedRpm / rotorGearRatio);
public static final double feederKv = 12.0 / (neoFreeSpeedRpm / feedGearRatio);

}
