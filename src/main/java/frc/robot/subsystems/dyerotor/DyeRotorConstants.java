package frc.robot.subsystems.dyerotor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class DyeRotorConstants {
    // --- CAN IDs ---
    public static final int rotorCanId = 29;
    public static final int feederCanId = 28;

    // --- Motor Configurations ---
    public static final boolean rotorIsInverted = false;
    public static final boolean feederIsInverted = true;
    public static final int rotorMotorCurrent = 40;
    public static final int feederMotorCurrent = 40;

    // --- Physical Dimensions & Mechanics ---
    public static final double ballsPerRotation = 8.0;
    public static final double fuelDiameterInches = 150/25.4; // 150mm in inches
    public static final double feedWheelRadiusInches = 1.625/2.0; // 1.625in diameter
    // Motor RPM = Mechanism RPM * Gear Ratio
    public static final double rotorGearRatio = 3.0*4.0*(72.0/16.0);
    public static final double feedGearRatio = 60.0 / 37.0;

    // --- Encoder & Conversion Factors ---
    public static final double rotorEncoderPositionFactor = 1.0 /rotorGearRatio;
    public static final double rotorEncoderVelocityFactor = 1.0 /rotorGearRatio;
    public static final double feederEncoderPositionFactor = 1.0/feedGearRatio;
    public static final double feederEncoderVelocityFactor = 1.0/feedGearRatio;

    // --- Control Loop Constants (PID & Feedforward) ---
    public static final double rotorKp = 0;
    public static final double rotorKi = 0;
    public static final double rotorKd = 0;
    public static final double feederKp = 0;
    public static final double feederKi = 0;
    public static final double feederKd = 0;
    public static final DCMotor rotorGearbox = DCMotor.getNeoVortex(1);
    public static final DCMotor feederGearbox = DCMotor.getNEO(1);
    public static final double rotorFreeSpeedRPM =
            Units.radiansPerSecondToRotationsPerMinute(rotorGearbox.freeSpeedRadPerSec);
    public static final double feederFreeSpeedRPM =
            Units.radiansPerSecondToRotationsPerMinute(feederGearbox.freeSpeedRadPerSec);
    // kV converts a target RPM into a 0.0 to 12.0 motor voltage command.
    // Formula: 12 volts / (MotorFreeSpeed / GearRatio)
    // This ensures that commanding the theoretical max mechanism RPM outputs exactly 12.0 (Max Voltage).
    public static final double rotorKv = 12.0 / (rotorFreeSpeedRPM / rotorGearRatio);
    public static final double feederKv = 12.0 / (feederFreeSpeedRPM / feedGearRatio);

    // --- Subsystem Behaviors & Targets ---
    // Throughput target used by runDyeRotor(true)
    public static final double defaultTargetBps = 16;
    public static final double overfeedRatio = 1.18; //Rate balls are fed realitive to the rotor speed. Based on Wildstang Calcs
}
