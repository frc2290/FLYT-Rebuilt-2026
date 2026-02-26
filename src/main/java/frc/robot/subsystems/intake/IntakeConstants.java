package frc.robot.subsystems.intake;

public class IntakeConstants {
    public enum IntakeSide {
        LEFT,
        RIGHT;

        public IntakeSide opposite() {
            return this == LEFT ? RIGHT : LEFT;
        }
    }

    public static final double inPosition = 0;
    public static final double outPosition = 90;
    public static final double positionBuffer = 5;

    public static final double rollerSpeed = 0.5;

    // public static final double odometryFrequency = 100.0;

    // tbd
    public static final int leftDriveCanId = 53;
    public static final int rightDriveCanId = 52;
    public static final int leftDeployCanId = 51;
    public static final int rightDeployCanId = 50;

    public static final int driveMotorCurrentLimit = 40;

    public static final boolean deployInverted = false;
    public static final boolean deployEncoderInverted = false;
    public static final int deployMotorCurrentLimit = 20;

    // i don't actually know what these values are; these need to be changed
    public static final double driveMotorReduction = 7 / 1;
    public static final double deployMotorReduction = 90 / 1;

    public static final double deployKp = 0.05;
    public static final double deployKi = 0.0;
    public static final double deployKd = 0.0;

    public static final double deploySimKp = 8.0;
    public static final double deploySimKi = 0.0;
    public static final double deploySimKd = 1.0;
}
