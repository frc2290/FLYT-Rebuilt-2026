package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.rlog.RLOGServer;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.math.util.Units.degreesToRadians;
import static edu.wpi.first.math.util.Units.feetToMeters;
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
        Logger.processInputs("Intake/Right", inputsRight);
        Logger.recordOutput("Intake/LeftIn", isIn(IntakeSide.LEFT));
        Logger.recordOutput("Intake/LeftOut", isOut(IntakeSide.LEFT));
        Logger.recordOutput("Intake/RightIn", isIn(IntakeSide.RIGHT));
        Logger.recordOutput("Intake/RightOut", isOut(IntakeSide.RIGHT));

        Logger.recordOutput("Intake/Components", new Pose3d[] {new Pose3d(0, getPosition(IntakeSide.LEFT) / outPosition * feetToMeters(1), 0, new Rotation3d()),
                                                                   new Pose3d(0, getPosition(IntakeSide.RIGHT) / outPosition * -feetToMeters(1), 0, new Rotation3d())});
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

    // when it is FULLY undeployed
    public boolean isIn(IntakeSide side) {
        return getPosition(side) < (inPosition + positionBuffer);
    }

    // when it is FULLY deployed
    public boolean isOut(IntakeSide side) {
        return getPosition(side) > (outPosition - positionBuffer);
    }

    public void deploy(IntakeSide side, boolean out) {
        getIntake(side).setDeployPosition(new Rotation2d(degreesToRadians(out ? outPosition : inPosition)));
    }

    public void driveRoller(IntakeSide side, double speed) {
        getIntake(side).setIntakeSpeed(speed);
    }

    public Command intakeIn(IntakeSide side) {
        return run(() -> {
            deploy(side, false);
        }).until(() -> {
            return isIn(side);
        }).andThen(() -> {
            // just in case
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

    public Command driveIntake() {
        return Commands.run(() -> {
            if (!isIn(IntakeSide.LEFT)) {
                driveRoller(IntakeSide.LEFT, rollerSpeed);
            }
            if (!isIn(IntakeSide.RIGHT)) {
                driveRoller(IntakeSide.RIGHT, rollerSpeed);
            }
            // driveRoller(IntakeSide.LEFT, isIn(IntakeSide.LEFT) ? 0 : rollerSpeed);
            // driveRoller(IntakeSide.RIGHT, isIn(IntakeSide.RIGHT) ? 0 : rollerSpeed);
        }).finallyDo(() -> {
                driveRoller(IntakeSide.LEFT, 0);
                driveRoller(IntakeSide.RIGHT, 0);
        });
    }
}
