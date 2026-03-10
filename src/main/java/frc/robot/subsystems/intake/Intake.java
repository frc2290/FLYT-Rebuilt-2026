package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;

import static edu.wpi.first.math.util.Units.feetToMeters;
import static edu.wpi.first.math.util.Units.radiansToDegrees;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import java.util.Optional;

public class Intake extends SubsystemBase {
    private final IntakeIO ioLeft;
    private final IntakeIOInputsAutoLogged inputsLeft = new IntakeIOInputsAutoLogged();
    private final IntakeIO ioRight;
    private final IntakeIOInputsAutoLogged inputsRight = new IntakeIOInputsAutoLogged();

    private double wowowowowoTicks = 0;

    private Optional<IntakeSide> outSide = Optional.empty();

    // Dashboard booleans to disable an intake side in a match
    // TODO IMPLEMENT
    private final LoggedNetworkBoolean leftDisabledDash = new LoggedNetworkBoolean("Left Intake Disabled", false);
    private final LoggedNetworkBoolean rightDisabledDash = new LoggedNetworkBoolean("Right Intake Disabled", false);

    public Intake(IntakeIO ioLeft, IntakeIO ioRight) {
        this.ioLeft = ioLeft;
        this.ioRight = ioRight;
    }

    @Override
    public void periodic() {
        ioLeft.updateInputs(inputsLeft);
        ioRight.updateInputs(inputsRight);
        Logger.processInputs("Intake/Left", inputsLeft);
        Logger.processInputs("Intake/Right", inputsRight);
        Logger.recordOutput("Intake/LeftIn", isIn(IntakeSide.LEFT));
        Logger.recordOutput("Intake/LeftOut", isOut(IntakeSide.LEFT));
        Logger.recordOutput("Intake/RightIn", isIn(IntakeSide.RIGHT));
        Logger.recordOutput("Intake/RightOut", isOut(IntakeSide.RIGHT));

        Logger.recordOutput("Intake/Components", new Pose3d[] {
                new Pose3d(0, getPosition(IntakeSide.LEFT) / outPosition * feetToMeters(1), 0, new Rotation3d()),
                new Pose3d(0, getPosition(IntakeSide.RIGHT) / outPosition * -feetToMeters(1), 0, new Rotation3d()) });

        if (!isIn(IntakeSide.LEFT) && !isIn(IntakeSide.RIGHT)) {
            // panic???
            
        }
    }

    private IntakeIO getIo(IntakeSide side) {
        // i am addicted to ternary operators
        return side == IntakeSide.LEFT ? ioLeft : ioRight;
    }

    public double getPosition(IntakeSide side) {
        // i am addicted to ternary operators pt 2
        return (side == IntakeSide.LEFT ? inputsLeft : inputsRight).deployPosition;
    }

    // when it is FULLY undeployed
    public boolean isIn(IntakeSide side) {
        return getPosition(side) < (inPosition + positionBuffer);
    }

    // when it is FULLY deployed
    public boolean isOut(IntakeSide side) {
        return getPosition(side) > (outPosition - positionBuffer);
    }

    public void deploy(IntakeSide side, boolean out) {
        getIo(side).setDeployPosition(out ? outPosition : inPosition);
    }

    public void driveRoller(IntakeSide side, double speed) {
        getIo(side).setIntakeSpeed(speed);
    }

    public Command intakeIn(IntakeSide side) {
        return run(() -> {
            outSide = Optional.empty();
            driveRoller(side, rollerSpeed);
            deploy(side, false);
        }).until(() -> {
            return isIn(side);
        }).andThen(() -> {
            driveRoller(side, 0);
        }, this);
    }

    public Command intakeOut(IntakeSide side) {
        return intakeIn(side.opposite()).andThen(run(() -> {
            // driveRoller(side, rollerSpeed);
            deploy(side, true);
        }).until(() -> {
            return isOut(side);
        }));
    }

    private void runIntake() {
        if (isOut(IntakeSide.LEFT)) {
            driveRoller(IntakeSide.LEFT, rollerSpeed);
        }
        if (isOut(IntakeSide.RIGHT)) {
            driveRoller(IntakeSide.RIGHT, rollerSpeed);
        }
    }

    private void stopIntake() {
        driveRoller(IntakeSide.LEFT, 0);
        driveRoller(IntakeSide.RIGHT, 0);
    }

    public Command startIntakeCommand() {
        return Commands.runOnce(this::runIntake);
    }

    public Command stopIntakeCommand() {
        return Commands.runOnce(this::stopIntake);
    }

    public Command runIntakeCommand() {
        return Commands.run(this::runIntake).finallyDo(this::stopIntake);
    }

    public Command wowowowowoIntake() {
        return run(() -> {
            wowowowowoTicks++;
            double angle = outPosition - (Math.sin(wowowowowoTicks / 15.0) / 2.0 + 0.5) * (outPosition * 0.85);
            outSide.ifPresent(side -> {
                driveRoller(side, rollerSpeed);
                getIo(side).setDeployPosition(radiansToDegrees(angle));
            });
        }).finallyDo(() -> {
            outSide.ifPresent(side -> {
                driveRoller(side, 0);
                getIo(side).setDeployPosition(outPosition);
            });
        });
    }
}