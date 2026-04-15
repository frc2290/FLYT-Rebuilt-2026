package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;

import static edu.wpi.first.math.util.Units.feetToMeters;
import static frc.robot.subsystems.intake.IntakeConstants.*;

import java.util.Optional;

public class Intake extends SubsystemBase {
    public enum ControlMode {
        VELOCITY,
        VOLTAGE
    }

    private final IntakeIO ioLeft;
    private final IntakeIOInputsAutoLogged inputsLeft = new IntakeIOInputsAutoLogged();
    private final IntakeIO ioRight;
    private final IntakeIOInputsAutoLogged inputsRight = new IntakeIOInputsAutoLogged();
    private final SysIdRoutine leftSysId;
    private final SysIdRoutine rightSysId;

    private double wowowowowoTicks = 0;
    private ControlMode leftControlMode = ControlMode.VELOCITY;
    private ControlMode rightControlMode = ControlMode.VELOCITY;
    private double leftCommandedVoltage = 0.0;
    private double rightCommandedVoltage = 0.0;

    private Optional<IntakeSide> outSide = Optional.empty();

    // Dashboard booleans to disable an intake side in a match
    // TODO IMPLEMENT
    //private final LoggedNetworkBoolean leftDisabledDash = new LoggedNetworkBoolean("Left Intake Disabled", false);
    //private final LoggedNetworkBoolean rightDisabledDash = new LoggedNetworkBoolean("Right Intake Disabled", false);

    public Intake(IntakeIO ioLeft, IntakeIO ioRight) {
        this.ioLeft = ioLeft;
        this.ioRight = ioRight;

        leftSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.75).per(Second),
                        Volts.of(11.0),
                        Seconds.of(16),
                        (state) -> Logger.recordOutput("Intake/LeftSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> leftCommandedVoltage = voltage.in(Volts),
                        null,
                        this,
                        "IntakeLeft"));

        rightSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.75).per(Second),
                        Volts.of(11.0),
                        Seconds.of(16),
                        (state) -> Logger.recordOutput("Intake/RightSysIdState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> rightCommandedVoltage = voltage.in(Volts),
                        null,
                        this,
                        "IntakeRight"));
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
        Logger.recordOutput("Intake/LeftControlMode", leftControlMode.toString());
        Logger.recordOutput("Intake/RightControlMode", rightControlMode.toString());

        Logger.recordOutput("Intake/Components", new Pose3d[] {
                new Pose3d(0, getLinearPosition(IntakeSide.LEFT), 0, new Rotation3d()),
                new Pose3d(0, -getLinearPosition(IntakeSide.RIGHT), 0, new Rotation3d()) });

        if (leftControlMode == ControlMode.VOLTAGE) {
            ioLeft.setIntakeVoltage(leftCommandedVoltage);
        }
        if (rightControlMode == ControlMode.VOLTAGE) {
            ioRight.setIntakeVoltage(rightCommandedVoltage);
        }

        if (!isIn(IntakeSide.LEFT) && !isIn(IntakeSide.RIGHT)) {
            // panic???
        }

        Robot.batteryLogger.reportCurrentUsage("Intake/Deploy/Left", inputsLeft.deployCurrentAmps, inputsLeft.deployAppliedVolts);
        Robot.batteryLogger.reportCurrentUsage("Intake/Deploy/Right", inputsRight.deployCurrentAmps, inputsRight.deployAppliedVolts);
        Robot.batteryLogger.reportCurrentUsage("Intake/Drive/Left", inputsLeft.driveCurrentAmps, inputsLeft.driveAppliedVolts);
        Robot.batteryLogger.reportCurrentUsage("Intake/Drive/Right", inputsRight.driveCurrentAmps, inputsRight.driveAppliedVolts);
    }

    private IntakeIO getIo(IntakeSide side) {
        // i am addicted to ternary operators
        return side == IntakeSide.LEFT ? ioLeft : ioRight;
    }

    public double getPosition(IntakeSide side) {
        // i am addicted to ternary operators pt 2
        return (side == IntakeSide.LEFT ? inputsLeft : inputsRight).deployPosition;
    }

    public double getLinearPosition(IntakeSide side) {
        return (getPosition(side) - IntakeConstants.inPosition) / (IntakeConstants.outPosition - IntakeConstants.inPosition) * feetToMeters(1.0);
    }

    // when it is FULLY undeployed
    public boolean isIn(IntakeSide side) {
        return getPosition(side) < (inPosition + positionBuffer);
    }

    // when it is FULLY deployed
    public boolean isOut(IntakeSide side) {
        return getPosition(side) > (outPosition - positionBuffer);
    }

    // when it was selected and is now out
    public boolean isSelected(IntakeSide side) {
        return outSide.equals(Optional.of(side));
    }

    public void deploy(IntakeSide side, boolean out) {
        getIo(side).setDeployPosition(out ? outPosition : inPosition, true);
    }

    public void driveRoller(IntakeSide side, double speed) {
        setControlMode(side, ControlMode.VELOCITY);
        getIo(side).setIntakeSpeed(speed);
    }

    /**
     * DO NOT USE this opens both intakes at once (for filming)
     * @return the command to supermegaoverride
     */
    public Command superMegaOverrideCommand() {
        return run(() -> {
            deploy(IntakeSide.LEFT, true);
            deploy(IntakeSide.RIGHT, true);
        }).until(() -> {
            return isOut(IntakeSide.LEFT) && isOut(IntakeSide.RIGHT);
        });
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
            outSide = Optional.of(side);
        }).until(() -> {
            return isOut(side);
        }));
    }

    public Command bothIn() {
        return run(() -> {
            outSide = Optional.empty();
            driveRoller(IntakeSide.LEFT, rollerSpeed);
            driveRoller(IntakeSide.RIGHT, rollerSpeed);
            deploy(IntakeSide.LEFT, false);
            deploy(IntakeSide.RIGHT, false);
        }).until(() -> {
            return isIn(IntakeSide.LEFT) && isIn(IntakeSide.RIGHT);
        }).andThen(() -> {
            driveRoller(IntakeSide.LEFT, 0);
            driveRoller(IntakeSide.RIGHT, 0);
        }, this);
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
        return startRun(() -> {
            wowowowowoTicks = 0;
        }, () -> {
            wowowowowoTicks++;
            double angle = outPosition - (Math.cos(wowowowowoTicks / 15.0) / -2.0 + 0.5) * (outPosition * 0.85);
            outSide.ifPresent(side -> {
                driveRoller(side, rollerSpeed);
                getIo(side).setDeployPosition(angle, true);
            });
        }).finallyDo(() -> {
            outSide.ifPresent(side -> {
                driveRoller(side, 0);
                getIo(side).setDeployPosition(outPosition, true);
            });
        });
    }

    /** Characterization sweep for the left or right deploy mechanism. */
    public Command agitateIntake(IntakeSide side) {
        Timer agitateTimer = new Timer();
        String logPrefix = side == IntakeSide.LEFT ? "Intake/Left" : "Intake/Right";
        double frequencyHz = 1.0;
        double agitateOutPosition = outPosition;
        double agitateInPosition = outPosition * 0.5;

        return startRun(() -> {
            agitateTimer.restart();
        }, () -> {
            double wave = 0.5 + 0.5 * Math.cos(2 * Math.PI * frequencyHz * agitateTimer.get());
            double angle = agitateInPosition + (wave * (agitateOutPosition - agitateInPosition));
            driveRoller(side, rollerSpeed/2);
            getIo(side).setDeployPosition(angle, false);
            Logger.recordOutput(logPrefix + "DeployCharacterizeTarget", angle);
        }).finallyDo(() -> {
            agitateTimer.stop();
            driveRoller(side, 0);
            getIo(side).setDeployPosition(outPosition, true);
        });
    }

    public Command syringeIntake(IntakeSide side) {
        return run(() -> {
            driveRoller(side, rollerSpeed/2);
            getIo(side).setDeployPosition(outPosition * 0.5);
        }).finallyDo(() -> {
            driveRoller(side, 0);
            getIo(side).setDeployPosition(outPosition);
        });
    }

    public Command agitateIntakeLeft() {
        return agitateIntake(IntakeSide.LEFT);
    }

    public Command agitateIntakeRight() {
        return agitateIntake(IntakeSide.RIGHT);
    }

    public ControlMode getControlMode(IntakeSide side) {
        return side == IntakeSide.LEFT ? leftControlMode : rightControlMode;
    }

    public void setControlMode(IntakeSide side, ControlMode controlMode) {
        ControlMode resolved = controlMode == null ? ControlMode.VELOCITY : controlMode;
        if (side == IntakeSide.LEFT) {
            leftControlMode = resolved;
            if (leftControlMode == ControlMode.VELOCITY) {
                leftCommandedVoltage = 0.0;
                ioLeft.setIntakeVoltage(0.0);
            }
        } else {
            rightControlMode = resolved;
            if (rightControlMode == ControlMode.VELOCITY) {
                rightCommandedVoltage = 0.0;
                ioRight.setIntakeVoltage(0.0);
            }
        }
    }

    /** Runs a quasistatic SysId test on the LEFT intake roller. */
    public Command sysIdQuasistaticLeftIntake(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            setControlMode(IntakeSide.LEFT, ControlMode.VOLTAGE);
            leftCommandedVoltage = 0.0;
        }).andThen(leftSysId.quasistatic(direction)).finallyDo(() -> {
            setControlMode(IntakeSide.LEFT, ControlMode.VELOCITY);
            leftCommandedVoltage = 0.0;
            stopIntake();
        });
    }

    /** Runs a dynamic SysId test on the LEFT intake roller. */
    public Command sysIdDynamicLeftIntake(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            setControlMode(IntakeSide.LEFT, ControlMode.VOLTAGE);
            leftCommandedVoltage = 0.0;
        }).andThen(leftSysId.dynamic(direction)).finallyDo(() -> {
            setControlMode(IntakeSide.LEFT, ControlMode.VELOCITY);
            leftCommandedVoltage = 0.0;
            stopIntake();
        });
    }

    /** Runs a quasistatic SysId test on the RIGHT intake roller. */
    public Command sysIdQuasistaticRightIntake(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            setControlMode(IntakeSide.RIGHT, ControlMode.VOLTAGE);
            rightCommandedVoltage = 0.0;
        }).andThen(rightSysId.quasistatic(direction)).finallyDo(() -> {
            setControlMode(IntakeSide.RIGHT, ControlMode.VELOCITY);
            rightCommandedVoltage = 0.0;
            stopIntake();
        });
    }

    /** Runs a dynamic SysId test on the RIGHT intake roller. */
    public Command sysIdDynamicRightIntake(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            setControlMode(IntakeSide.RIGHT, ControlMode.VOLTAGE);
            rightCommandedVoltage = 0.0;
        }).andThen(rightSysId.dynamic(direction)).finallyDo(() -> {
            setControlMode(IntakeSide.RIGHT, ControlMode.VELOCITY);
            rightCommandedVoltage = 0.0;
            stopIntake();
        });
    }
}
