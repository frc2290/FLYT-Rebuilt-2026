package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public class IntakeConstants {
    // --- Enums ---
    public enum IntakeSide {
        LEFT,
        RIGHT;

        public IntakeSide opposite() {
            return this == LEFT ? RIGHT : LEFT;
        }
    }

    // --- CAN IDs ---
    public static final int leftDriveCanId = 53;
    public static final int rightDriveCanId = 52;
    public static final int leftDeployCanId = 51;
    public static final int rightDeployCanId = 50;

    // --- Motor Configurations ---
    public static final boolean deployInverted = false;
    public static final int driveMotorCurrentLimit = 40;
    public static final int deployMotorCurrentLimit = 20;

    // --- Physical Dimensions & Mechanics ---
    public static final double driveMotorReduction = (11.0 / 29.0) * (18.0 / 20.0);
    public static final double deployMotorReduction = 90 / 1;
    public static final double rollerDiameterMeters = 1.25 * 0.0254;

    // --- Encoder & Conversion Factors ---
    public static final boolean deployEncoderInverted = false;
    public static final double rollerEncoderPositionFactor = driveMotorReduction * (Math.PI * rollerDiameterMeters);
    public static final double rollerEncoderVelocityFactor = rollerEncoderPositionFactor / 60.0;

    // --- Control Loop Constants (PID & Feedforward) ---
    public static final double rollerKp = 0.0;
    public static final double rollerKi = 0.0;
    public static final double rollerKd = 0.0;
    public static final DCMotor rollerGearbox = DCMotor.getNEO(1);
    public static final double rollerFreeSpeedRPM =
            Units.radiansPerSecondToRotationsPerMinute(rollerGearbox.freeSpeedRadPerSec);
    public static final double rollerKv = 12.0 / (rollerFreeSpeedRPM * rollerEncoderVelocityFactor);
    public static final double deployKp = 0.05;
    public static final double deployKi = 0.0;
    public static final double deployKd = 0.0;

    // --- Simulation Constants ---
    public static final double deploySimKp = 8.0;
    public static final double deploySimKi = 0.0;
    public static final double deploySimKd = 1.0;

    // --- Subsystem Behaviors & Targets ---
    public static final double inPosition = 0;
    public static final double outPosition = 90;
    public static final double positionBuffer = 5;
    // Intake roller surface speed command in m/s.
    public static final double rollerSpeed = 3;
}
