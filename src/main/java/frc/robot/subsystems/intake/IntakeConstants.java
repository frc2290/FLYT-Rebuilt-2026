package frc.robot.subsystems.intake;

public class IntakeConstants {
    enum IntakeSide {
        LEFT,
        RIGHT;

        public IntakeSide opposite() {
            return this == LEFT ? RIGHT : LEFT;
        }
    }

    public static final double inPosition = 0;
    public static final double outPosition = 90;
    public static final double positionBuffer = 5;

    public static final double rollerVelocity = 10;
}
