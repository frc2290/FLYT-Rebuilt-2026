package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static frc.robot.subsystems.intake.IntakeConstants.*;

public class Intake extends SubsystemBase {
    private final IntakeIO ioLeft;
    private final IntakeIOInputsAutoLogged inputsLeft = new IntakeIOInputsAutoLogged();
    private final IntakeIO ioRight;
    private final IntakeIOInputsAutoLogged inputsRight = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO ioLeft, IntakeIO ioRight) {
        this.ioLeft = ioLeft;
        this.ioRight = ioRight;
    }

    @Override
    public void periodic() {
        ioLeft.updateInputs(inputsLeft);
        ioRight.updateInputs(inputsRight);
        Logger.processInputs("Intake/Left", inputsLeft);
        Logger.processInputs("Intake/Right", inputsLeft);
    }

    private IntakeIO getIntake(IntakeSide side) {
        // i am addicted to ternary operators
        return side == IntakeSide.LEFT ? ioLeft : ioRight;
    }

    public double getPosition(IntakeSide side) {
        if (side == IntakeSide.LEFT) {
            return inputsLeft.deployPosition.getDegrees();
        } else {
            return inputsRight.deployPosition.getDegrees();
        }
    }

    public boolean isIn(IntakeSide side) {
        return getPosition(side) < (inPosition + positionBuffer);
    }

    public boolean isOut(IntakeSide side) {
        return getPosition(side) > (outPosition - positionBuffer);
    }

    public void deploy(IntakeSide side, boolean out) {
        getIntake(side).setDeployPosition(new Rotation2d(degreesToRadians(out ? outPosition : inPosition)));
    }

    public void driveRoller(IntakeSide side, double vel) {
        getIntake(side).setIntakeVelocity(vel);
    }

    public Command intakeIn(IntakeSide side) {
        return run(() -> {
            deploy(side, false);
        }).until(() -> {
            return isIn(side);
        }).andThen(() -> {
            driveRoller(side, 0);
        }, this);
    }

    public Command intakeOut(IntakeSide side) {
        return intakeIn(side.opposite()).andThen(run(() -> {
            driveRoller(side, rollerVelocity);
            deploy(side, true);
        }).until(() -> {
            return isOut(side);
        }));
    }
}
