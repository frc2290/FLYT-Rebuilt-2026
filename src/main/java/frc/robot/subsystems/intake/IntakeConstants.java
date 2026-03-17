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
    public static final int leftDriveCanId = 52;
    public static final int rightDriveCanId = 53;
    public static final int leftDeployCanId = 51;
    public static final int rightDeployCanId = 50;

    // --- Motor Configurations ---
    public static final boolean deployInverted = false;
    public static final int driveMotorCurrentLimit = 40;
    public static final int deployMotorCurrentLimit = 40;

    // --- Physical Dimensions & Mechanics ---
    public static final double driveMotorReduction = (11.0 / 29.0) * (18.0 / 25.0) * (30.0 / 20.0) / 5;
    public static final double deployMotorToEncoderReduction = 84.0 / 7.0;
    public static final double deployEncoderToLinkageReduction = 48.0 / 16.0;
    public static final double deployMotorReduction =
            deployMotorToEncoderReduction * deployEncoderToLinkageReduction;
    public static final double rollerDiameterMeters = 1.25 * 0.0254;

    // --- Encoder & Conversion Factors ---
    public static final boolean deployEncoderInverted = false;
    public static final double deployEncoderPositionFactor =
            (1.0 / deployEncoderToLinkageReduction) * 360.0;
    public static final double deployEncoderVelocityFactor = deployEncoderPositionFactor / 60.0;
    public static final double rollerEncoderPositionFactor = driveMotorReduction * (Math.PI * rollerDiameterMeters);
    public static final double rollerEncoderVelocityFactor = rollerEncoderPositionFactor / 60.0;

    // --- Control Loop Constants (PID & Feedforward) ---
    public static final DCMotor rollerGearbox = DCMotor.getNeoVortex(1);
    public static final double rollerFreeSpeedRPM =
            Units.radiansPerSecondToRotationsPerMinute(rollerGearbox.freeSpeedRadPerSec);
    public static final double rollerKv = 12.0 / (rollerFreeSpeedRPM * rollerEncoderVelocityFactor);
    public static final double rollerKp = 0.5;
    public static final double rollerKi = 0.0;
    public static final double rollerKd = 0.0;
    public static final DCMotor deployGearbox = DCMotor.getNeoVortex(1);
    public static final double deployFreeSpeedRPM =
            Units.radiansPerSecondToRotationsPerMinute(deployGearbox.freeSpeedRadPerSec);
    public static final double deployKv = 12.0
            / ((deployFreeSpeedRPM / deployMotorToEncoderReduction) * deployEncoderVelocityFactor);
    public static final double deployKp = 0.025;
    public static final double deployKi = 0.0;
    public static final double deployKd = 0.0;

    // --- Simulation Constants ---
    public static final double deploySimKp = 8.0;
    public static final double deploySimKi = 0.0;
    public static final double deploySimKd = 1.0;

    // --- Subsystem Behaviors & Targets ---
    public static final double inPosition = 5;
    public static final double outPosition = 86;
    public static final double positionBuffer = 5;
    // Intake roller surface speed command in m/s.
    public static final double rollerSpeed = 3;
    public static final double leftZeroOffsetAdj = 17.2;
    public static final double rightZeroOffsetAdj = 16.3;
}
