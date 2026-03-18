package frc.robot.subsystems.dyerotor;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.dyerotor.DyeRotorConstants.*;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;

public class DyeRotor extends SubsystemBase {
    public enum ControlMode {
        VELOCITY,
        VOLTAGE
    }

    private final DyeRotorIO io;
    private final DyeRotorIOInputsAutoLogged inputs = new DyeRotorIOInputsAutoLogged();
    private final SysIdRoutine rotorSysId;
    private final SysIdRoutine feederSysId;
    private final Debouncer rotorAtSpeedDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
    private final Debouncer feederAtSpeedDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private final Timer jamTimer = new Timer();
    private final Timer backdriveTimer = new Timer();

    private boolean runDyeRotor = false;
    private boolean backdriving = false;
    private ControlMode rotorControlMode = ControlMode.VELOCITY;
    private ControlMode feederControlMode = ControlMode.VELOCITY;
    private double rotorCommandedVoltage = 0.0;
    private double feederCommandedVoltage = 0.0;
    private double currentTargetRotorRps = 0.0;
    private double currentTargetFeederRps = 0.0;
    private boolean rotorAtSpeedState = false;
    private boolean feederAtSpeedState = false;

    public DyeRotor(DyeRotorIO io) {
        this.io = io;

        rotorSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.75).per(Second),
                        Volts.of(11.5),
                        Seconds.of(16),
                        (state) -> Logger.recordOutput("DyeRotor/SysIdRotorState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> rotorCommandedVoltage = voltage.in(Volts),
                        null,
                        this));

        feederSysId = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Volts.of(0.5).per(Second),
                        Volts.of(8.5),
                        Seconds.of(24),
                        (state) -> Logger.recordOutput("DyeRotor/SysIdFeederState", state.toString())),
                new SysIdRoutine.Mechanism(
                        (voltage) -> feederCommandedVoltage = voltage.in(Volts),
                        null,
                        this));
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        rotorAtSpeedState =
                rotorAtSpeedDebouncer.calculate(rotorControlMode == ControlMode.VELOCITY && io.rotorAtSetpoint());
        feederAtSpeedState =
                feederAtSpeedDebouncer.calculate(feederControlMode == ControlMode.VELOCITY && io.feederAtSetpoint());
        Logger.processInputs("DyeRotor", inputs);
        OptionalDouble measuredOverfeedRatio = getMeasuredOverfeedRatio();
        Logger.recordOutput("DyeRotor/RotorControlMode", rotorControlMode.toString());
        Logger.recordOutput("DyeRotor/FeederControlMode", feederControlMode.toString());
        Logger.recordOutput("DyeRotor/MeasuredOverfeedRatio", measuredOverfeedRatio.orElse(Double.NaN));
        Logger.recordOutput("DyeRotor/TargetRotorRps", currentTargetRotorRps);
        Logger.recordOutput("DyeRotor/TargetFeederRps", currentTargetFeederRps);
        Logger.recordOutput("DyeRotor/RotorAtSpeed", rotorAtSpeedState);
        Logger.recordOutput("DyeRotor/FeederAtSpeed", feederAtSpeedState);
        Logger.recordOutput("DyeRotor/AtFeedingSpeeds", isAtFeedingSpeeds());

        if (rotorControlMode == ControlMode.VOLTAGE || feederControlMode == ControlMode.VOLTAGE) {
            io.setRotorVoltage(rotorControlMode == ControlMode.VOLTAGE ? rotorCommandedVoltage : 0.0);
            io.setFeederVoltage(feederControlMode == ControlMode.VOLTAGE ? feederCommandedVoltage : 0.0);
            stopDyeRotor();
            return;
        }

        if (!runDyeRotor) {
            io.setRotorSpeed(0);
            io.setFeederSpeed(0);
            stopDyeRotor();
            return;
        }

        if (backdriving) {
            setTargetBPS(-defaultTargetBps);
            if (backdriveTimer.hasElapsed(backdriveSeconds)) {
                backdriving = false;
                backdriveTimer.stop();
                backdriveTimer.reset();
                jamTimer.stop();
                jamTimer.reset();
            }
            return;
        }

        double targetRotorRps = defaultTargetBps / ballsPerRotation;
        boolean atTargetSpeed = Math.abs(inputs.rotorEncoderRPM) >= (targetRotorRps * atSpeedRatio);

        if (!atTargetSpeed) {
            if (!jamTimer.isRunning()) {
                jamTimer.reset();
                jamTimer.start();
            }

            if (jamTimer.hasElapsed(jamDetectSeconds)) {
                jamTimer.stop();
                jamTimer.reset();
                backdriveTimer.reset();
                backdriveTimer.start();
                io.resetIntegrator();
                backdriving = true;
            } else {
                setTargetBPS(defaultTargetBps);
            }
        } else {
            jamTimer.stop();
            jamTimer.reset();
            setTargetBPS(defaultTargetBps);
        }

        Robot.batteryLogger.reportCurrentUsage("DyeRotor/Feeder", inputs.feederCurrentAmps);
        Robot.batteryLogger.reportCurrentUsage("DyeRotor/Rotor", inputs.rotorCurrentAmps);
    }

    public void runDyeRotor(boolean run) {
        this.runDyeRotor = run;
        if (!run) {
            stopDyeRotor();
        }
        // backdriving = false;
        // jamTimer.stop();
        // jamTimer.reset();
        // backdriveTimer.stop();
        // backdriveTimer.reset();

        // if (!run) {
        //     io.setRotorSpeed(0);
        //     io.setFeederSpeed(0);
        // }
    }

    public Command runDyeRotorCommand(boolean run) {
        if (run) {
            return runDyeRotorAutoCommand();
        }
        return Commands.runOnce(() -> runDyeRotor(false), this);
    }

    public Command runDyeRotorAutoCommand() {
        return Commands.startEnd(() -> runDyeRotor(true), () -> runDyeRotor(false), this);
    }

    public void resetIntegrator() {
        io.resetIntegrator();
    }

    /**
     * Calculates and commands the motor speeds required to hit a target throughput.
     *
     * @param targetBPS target balls per second
     */
    public void setTargetBPS(double targetBPS) {
        // 1) DYE ROTOR KINEMATICS
        double rotorSpeed = targetBPS / ballsPerRotation;

        // 2) FEED WHEEL KINEMATICS (PURE ROLLING + OVERFEED)
        double feedSpeed = calculateFeedSpeedRps(rotorSpeed);

        // 3) COMMAND MECHANISM REV/SEC DIRECTLY
        // Spark encoder conversion factors apply motor->mechanism gear ratio scaling.
        currentTargetRotorRps = rotorSpeed;
        currentTargetFeederRps = feedSpeed;
        io.setRotorSpeed(rotorSpeed);
        io.setFeederSpeed(feedSpeed);
    }

    public void driveRotor(double speed) {
        setRotorControlMode(ControlMode.VELOCITY);
        currentTargetRotorRps = speed;
        io.setRotorSpeed(speed);
    }

    public void driveFeeder(double speed) {
        setFeederControlMode(ControlMode.VELOCITY);
        currentTargetFeederRps = speed;
        io.setFeederSpeed(speed);
    }

    public double getRotorSpeed() {
        return inputs.rotorEncoderRPM;
    }

    public double getFeederSpeed() {
        return inputs.feederEncoderRPM;
    }

    /**
     * Returns measured overfeed ratio from current rotor/feeder rev/sec.
     *
     * <p>Empty when rotor speed is too low for a stable ratio estimate.
     */
    public OptionalDouble getMeasuredOverfeedRatio() {
        double rotorRps = inputs.rotorEncoderRPM;
        if (Math.abs(rotorRps) < minRotorRpsForOverfeed) {
            return OptionalDouble.empty();
        }

        // setTargetBPS() commands feederSpeed = feedSpeedAbs - rotorSpeed.
        // Reconstruct feedSpeedAbs from measured speeds.
        double feedSpeedAbsRps = inputs.feederEncoderRPM + rotorRps;
        double feedMultiplierMeasured = Math.abs(feedSpeedAbsRps / rotorRps);
        double measuredOverfeedRatio = feedMultiplierMeasured
                * (Math.PI * feedWheelRadiusInches)
                / (ballsPerRotation * fuelDiameterInches);

        if (!Double.isFinite(measuredOverfeedRatio)) {
            return OptionalDouble.empty();
        }
        return OptionalDouble.of(measuredOverfeedRatio);
    }

    public ControlMode getRotorControlMode() {
        return rotorControlMode;
    }

    public void setRotorControlMode(ControlMode controlMode) {
        rotorControlMode = controlMode == null ? ControlMode.VELOCITY : controlMode;
        if (rotorControlMode == ControlMode.VELOCITY) {
            rotorCommandedVoltage = 0.0;
            io.setRotorVoltage(0.0);
        } else {
            currentTargetRotorRps = 0.0;
        }
    }

    public ControlMode getFeederControlMode() {
        return feederControlMode;
    }

    public void setFeederControlMode(ControlMode controlMode) {
        feederControlMode = controlMode == null ? ControlMode.VELOCITY : controlMode;
        if (feederControlMode == ControlMode.VELOCITY) {
            feederCommandedVoltage = 0.0;
            io.setFeederVoltage(0.0);
        } else {
            currentTargetFeederRps = 0.0;
        }
    }

    /** Runs a quasistatic SysId test on the DyeRotor rotor. */
    public Command sysIdQuasistaticRotor(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            runDyeRotor(false);
            setRotorControlMode(ControlMode.VOLTAGE);
            rotorCommandedVoltage = 0.0;
        }).andThen(rotorSysId.quasistatic(direction)).finallyDo(() -> {
            setRotorControlMode(ControlMode.VELOCITY);
            rotorCommandedVoltage = 0.0;
        });
    }

    /** Runs a dynamic SysId test on the DyeRotor rotor. */
    public Command sysIdDynamicRotor(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            runDyeRotor(false);
            setRotorControlMode(ControlMode.VOLTAGE);
            rotorCommandedVoltage = 0.0;
        }).andThen(rotorSysId.dynamic(direction)).finallyDo(() -> {
            setRotorControlMode(ControlMode.VELOCITY);
            rotorCommandedVoltage = 0.0;
        });
    }

    /** Runs a quasistatic SysId test on the DyeRotor feeder. */
    public Command sysIdQuasistaticFeeder(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            runDyeRotor(false);
            setFeederControlMode(ControlMode.VOLTAGE);
            feederCommandedVoltage = 0.0;
        }).andThen(feederSysId.quasistatic(direction)).finallyDo(() -> {
            setFeederControlMode(ControlMode.VELOCITY);
            feederCommandedVoltage = 0.0;
        });
    }

    /** Runs a dynamic SysId test on the DyeRotor feeder. */
    public Command sysIdDynamicFeeder(SysIdRoutine.Direction direction) {
        return runOnce(() -> {
            runDyeRotor(false);
            setFeederControlMode(ControlMode.VOLTAGE);
            feederCommandedVoltage = 0.0;
        }).andThen(feederSysId.dynamic(direction)).finallyDo(() -> {
            setFeederControlMode(ControlMode.VELOCITY);
            feederCommandedVoltage = 0.0;
        });
    }

    public boolean isRotorAtSpeed() {
        return rotorAtSpeedState;
    }

    public boolean isFeederAtSpeed() {
        return feederAtSpeedState;
    }

    public boolean isAtFeedingSpeeds() {
        return isRotorAtSpeed() && isFeederAtSpeed();
    }

    private void stopDyeRotor() {
        currentTargetRotorRps = 0.0;
        currentTargetFeederRps = 0.0;
    }

    private static double calculateFeedSpeedRps(double rotorSpeedRps) {
        double feedMultiplier =
                (ballsPerRotation * fuelDiameterInches * overfeedRatio) / (Math.PI * feedWheelRadiusInches);
        double feedSpeedAbs = rotorSpeedRps * feedMultiplier;
        return feedSpeedAbs - rotorSpeedRps;
    }

}
