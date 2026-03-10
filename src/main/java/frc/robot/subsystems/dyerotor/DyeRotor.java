package frc.robot.subsystems.dyerotor;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.subsystems.dyerotor.DyeRotorConstants.*;

import java.util.OptionalDouble;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class DyeRotor extends SubsystemBase {
    public enum ControlMode {
        VELOCITY,
        VOLTAGE
    }

    private static final double JAM_DETECT_SECONDS = 0.25;
    private static final double BACKDRIVE_SECONDS = 0.25;
    private static final double AT_SPEED_RATIO = 0.95;

    private final DyeRotorIO io;
    private final DyeRotorIOInputsAutoLogged inputs = new DyeRotorIOInputsAutoLogged();
    private final SysIdRoutine rotorSysId;
    private final SysIdRoutine feederSysId;

    private final Timer jamTimer = new Timer();
    private final Timer backdriveTimer = new Timer();

    private boolean runDyeRotor = false;
    private boolean backdriving = false;
    private ControlMode rotorControlMode = ControlMode.VELOCITY;
    private ControlMode feederControlMode = ControlMode.VELOCITY;
    private double rotorCommandedVoltage = 0.0;
    private double feederCommandedVoltage = 0.0;

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
        Logger.processInputs("DyeRotor", inputs);
        OptionalDouble measuredOverfeedRatio = getMeasuredOverfeedRatio();
        Logger.recordOutput("DyeRotor/RotorControlMode", rotorControlMode.toString());
        Logger.recordOutput("DyeRotor/FeederControlMode", feederControlMode.toString());
        Logger.recordOutput("DyeRotor/MeasuredOverfeedRatio", measuredOverfeedRatio.orElse(Double.NaN));

        if (rotorControlMode == ControlMode.VOLTAGE || feederControlMode == ControlMode.VOLTAGE) {
            io.setRotorVoltage(rotorControlMode == ControlMode.VOLTAGE ? rotorCommandedVoltage : 0.0);
            io.setFeederVoltage(feederControlMode == ControlMode.VOLTAGE ? feederCommandedVoltage : 0.0);
            return;
        }

        if (!runDyeRotor) {
            io.setRotorSpeed(0);
            io.setFeederSpeed(0);
            return;
        }

        if (backdriving) {
            setTargetBPS(-defaultTargetBps);
            if (backdriveTimer.hasElapsed(BACKDRIVE_SECONDS)) {
                backdriving = false;
                backdriveTimer.stop();
                backdriveTimer.reset();
                jamTimer.stop();
                jamTimer.reset();
            }
            return;
        }

        double targetRotorRpm = (defaultTargetBps * 60.0) / ballsPerRotation;
        boolean atTargetSpeed = Math.abs(inputs.rotorEncoderRPM) >= (targetRotorRpm * AT_SPEED_RATIO);

        if (!atTargetSpeed) {
            if (!jamTimer.isRunning()) {
                jamTimer.reset();
                jamTimer.start();
            }

            if (jamTimer.hasElapsed(JAM_DETECT_SECONDS)) {
                jamTimer.stop();
                jamTimer.reset();
                backdriveTimer.reset();
                backdriveTimer.start();
                backdriving = true;
            } else {
                setTargetBPS(defaultTargetBps);
            }
        } else {
            jamTimer.stop();
            jamTimer.reset();
            setTargetBPS(defaultTargetBps);
        }
    }

    public void runDyeRotor(boolean run) {
        this.runDyeRotor = run;
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

    /**
     * Calculates and commands the motor speeds required to hit a target throughput.
     *
     * @param targetBPS target balls per second
     */
    public void setTargetBPS(double targetBPS) {
        // 1) DYE ROTOR KINEMATICS
        double rotorSpeed = (targetBPS * 60.0) / ballsPerRotation;

        // 2) FEED WHEEL KINEMATICS (PURE ROLLING + OVERFEED)
        double feedMultiplier =
                (ballsPerRotation * fuelDiameterInches * overfeedRatio) / (Math.PI * feedWheelRadiusInches);
        double feedSpeedAbs = rotorSpeed * feedMultiplier;
        double feedSpeed = feedSpeedAbs - rotorSpeed;

        // 3) COMMAND MECHANISM RPM DIRECTLY
        // Spark encoder conversion factors apply motor->mechanism gear ratio scaling.
        io.setRotorSpeed(rotorSpeed);
        io.setFeederSpeed(feedSpeed);
    }

    public void driveRotor(double speed) {
        setRotorControlMode(ControlMode.VELOCITY);
        io.setRotorSpeed(speed);
    }

    public void driveFeeder(double speed) {
        setFeederControlMode(ControlMode.VELOCITY);
        io.setFeederSpeed(speed);
    }

    public double getRotorSpeed() {
        return inputs.rotorEncoderRPM;
    }

    public double getFeederSpeed() {
        return inputs.feederEncoderRPM;
    }

    /**
     * Returns measured overfeed ratio from current rotor/feeder RPM.
     *
     * <p>Empty when rotor speed is too low for a stable ratio estimate.
     */
    public OptionalDouble getMeasuredOverfeedRatio() {
        double rotorRpm = inputs.rotorEncoderRPM;
        if (Math.abs(rotorRpm) < minRotorRpmForOverfeed) {
            return OptionalDouble.empty();
        }

        // setTargetBPS() commands feederSpeed = feedSpeedAbs - rotorSpeed.
        // Reconstruct feedSpeedAbs from measured speeds.
        double feedSpeedAbsRpm = inputs.feederEncoderRPM + rotorRpm;
        double feedMultiplierMeasured = Math.abs(feedSpeedAbsRpm / rotorRpm);
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

}
