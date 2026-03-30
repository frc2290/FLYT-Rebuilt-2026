package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;
import static frc.utils.SparkUtil.*;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.utils.DualEncoderUnwrapper;

public class TurretIOSpark implements TurretIO {

    // Harware objects
    private final SparkBase turretSpark;
    private final SparkBase hoodSpark;
    private final SparkBase flywheel1Spark; // later change to bottom/top
    private final SparkBase flywheel2Spark;

    // Fo turret
    private RelativeEncoder turnRelEncoder; // Experimental for using one encoder for two things at the same time
    private DutyCycleEncoder turnEncoder1;
    private DutyCycleEncoder turnEncoder2;
    // private AbsoluteEncoder turnEncoder2;
    private AbsoluteEncoder hoodEncoder;
    private RelativeEncoder flywheel1Encoder;
    private RelativeEncoder flywheel2Encoder;

    // Closed loop controllers
    private final SparkClosedLoopController turretController;
    private final SparkClosedLoopController hoodController;
    private final SparkClosedLoopController flywheelController1;
    private final SparkClosedLoopController flywheelController2;

    private final Debouncer turretConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer hoodConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer flywheelConnectedDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    //

    // Global variables
    private boolean isTurretHomed = false;
    private double turretAngle = 0;
    private double turretSpeed = 0;
    private double turretHoodAngle = 10;
    private double turretAngleSetpoint = 0;
    private double shooterVoltageCommand = 0.0;

    public TurretIOSpark() {

        // Create motor controllers
        turretSpark = new SparkFlex(turretCanId, MotorType.kBrushless);
        hoodSpark = new SparkMax(hoodCanId, MotorType.kBrushless);
        flywheel1Spark = new SparkFlex(flywheel1CanId, MotorType.kBrushless);
        flywheel2Spark = new SparkFlex(flywheel2CanId, MotorType.kBrushless);

        // Setup encoders
        flywheel1Encoder = flywheel1Spark.getEncoder();
        flywheel2Encoder = flywheel2Spark.getEncoder();
        turnRelEncoder = turretSpark.getEncoder();
        hoodEncoder = hoodSpark.getAbsoluteEncoder();

        // Setup RoboRio Absolute encoders
        // Initializes duty cycle encoders on DIO pins 0 and 1.
        // get() returns position in rotations [0, 1].
        turnEncoder1 = new DutyCycleEncoder(8, 1, 0.6644293666107342); // Large
        turnEncoder2 = new DutyCycleEncoder(9, 1, 0.3600550840013771); // Small

        // Setup Controllers
        turretController = turretSpark.getClosedLoopController();
        hoodController = hoodSpark.getClosedLoopController();
        flywheelController1 = flywheel1Spark.getClosedLoopController();
        flywheelController2 = flywheel2Spark.getClosedLoopController();

        // Turret parameters
        var turretConfig = new SparkFlexConfig();
        turretConfig
                .inverted(turretIsInverted)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(turretMotorCurrent)
                .voltageCompensation(12.0);
        turretConfig.encoder
                .positionConversionFactor(turretEncoderPositionFactor)
                .velocityConversionFactor(turretEncoderVelocityFactor)
                .quadratureAverageDepth(2);
        turretConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(turretKp, turretKi, turretKd);
        turretConfig.closedLoop.feedForward.kV(turretTheoreticalKv);
        turretConfig.closedLoop.maxMotion
                // Calculated for a 0.5s, 86-degree move
                .cruiseVelocity(2600*4)
                .maxAcceleration(12000*4)
                // Keep this loose during tuning to prevent premature profile regeneration
                .allowedProfileError(5);
        REVLibError turretErr = turretSpark.configure(
                turretConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        if (turretErr != REVLibError.kOk) {
            System.err.println("TURRET CONFIG FAILED: " + turretErr.name());
        }

        // Hood Config
        var hoodConfig = new SparkMaxConfig();
        hoodConfig
                .inverted(hoodIsInverted)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(hoodMotorCurrent)
                .voltageCompensation(12.0);
        hoodConfig.absoluteEncoder
                .inverted(hoodEncoderInverted)
                .zeroOffset(hoodEncoderZeroOffset)
                .positionConversionFactor(hoodEncoderPositionFactor)
                .velocityConversionFactor(hoodEncoderVelocityFactor)
                .averageDepth(2)
                .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
        hoodConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(hoodKp, hoodKi, hoodKd);
        // hoodConfig.closedLoop.feedForward.kV(hoodKv);
        REVLibError hoodErr = hoodSpark.configure(
                hoodConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        if (hoodErr != REVLibError.kOk) {
            System.err.println("HOOD CONFIG FAILED: " + hoodErr.name());
        }

        // Flywheel base config (shared safety/hardware settings)
        var flywheelBaseConfig = new SparkFlexConfig();
        flywheelBaseConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(flywheelMotorCurrent);
        // .voltageCompensation(12.0);

        // Flywheel leader config
        var flywheelLeaderConfig = new SparkFlexConfig();
        flywheelLeaderConfig
                .apply(flywheelBaseConfig)
                .inverted(flywheelIsInverted);
        flywheelLeaderConfig.encoder
                .positionConversionFactor(flywheelEncoderPositionFactor)
                .velocityConversionFactor(flywheelEncoderVelocityFactor)
                .quadratureMeasurementPeriod(5)
                .quadratureAverageDepth(1);
        flywheelLeaderConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(flywheelKp, flywheelKi, flywheelKd);
        flywheelLeaderConfig.closedLoop.feedForward.kV(flywheelKv);
        flywheelLeaderConfig.closedLoop.feedForward.kS(flywheelKs);

        // Flywheel follower config
        var flywheelFollowerConfig = new SparkFlexConfig();
        flywheelFollowerConfig
                .apply(flywheelBaseConfig)
                .inverted(!flywheelIsInverted);
        flywheelFollowerConfig.encoder
                .positionConversionFactor(flywheelEncoderPositionFactor)
                .velocityConversionFactor(flywheelEncoderVelocityFactor)
                .quadratureMeasurementPeriod(5)
                .quadratureAverageDepth(1);
        flywheelFollowerConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(flywheelKp, flywheelKi, flywheelKd);
        flywheelFollowerConfig.closedLoop.feedForward.kV(flywheelKv);
        flywheelFollowerConfig.closedLoop.feedForward.kS(flywheelKs);

        REVLibError flywheel1Err = flywheel1Spark.configure(
                flywheelLeaderConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        if (flywheel1Err != REVLibError.kOk) {
            System.err.println("FLYWHEEL1 CONFIG FAILED: " + flywheel1Err.name());
        }
        REVLibError flywheel2Err = flywheel2Spark.configure(
                flywheelFollowerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        if (flywheel2Err != REVLibError.kOk) {
            System.err.println("FLYWHEEL2 CONFIG FAILED: " + flywheel2Err.name());
        }

        // this is very important line that checks turret position through absolute
        // encoders then sets relative encoder to same pose
        // turnRelEncoder.setPosition(calculateAbsoluteTurretAngle());
        // turnRelEncoder.setPosition(0.0);
    }

    // Absolute dual encoder position of turret estimator master piece
    // Also I think this function should be run only once to set position for
    // relative encoder
    private double calculateAbsoluteTurretAngle() {
        // Ratios: Turn mechanism gear / encoder pulleys.
        // Limits: Assumes the turret starts within +/- rangeTurret/2 rotations of
        // mechanism travel.
        DualEncoderUnwrapper unwrapper = new DualEncoderUnwrapper(
                numTeethTurret / numTeethPulley1,
                numTeethTurret / numTeethPulley2,
                -rangeTurret / 2.0,
                rangeTurret / 2.0);

        // DutyCycleEncoder get() already returns rotations in [0, 1).
        double abs1 = turnEncoder1.get();
        double abs2 = turnEncoder2.get();

        DualEncoderUnwrapper.UnwrapResult result = unwrapper.unwrap(abs1, abs2);

        if (result.status == DualEncoderUnwrapper.Status.OK) {
            return result.position * 360.0; // Convert mechanism rotations to degrees
        } else {
            System.err.println(
                    "Turret Unwrap Failed. Status: " + result.status + " | Error: " + result.error);
            return Double.NaN;
        }
    }

    /**
     * Get turret pose
     * 
     * @return
     */
    private double getTurretPos() {
        return turnRelEncoder.getPosition();
    }

    // I think this is used for logger pro to keep track of things?
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // Delayed absolute seeding: only latch when both absolute encoders are
        // connected.
        if (!isTurretHomed && turnEncoder1.isConnected() && turnEncoder2.isConnected()) {
            double startPos = calculateAbsoluteTurretAngle();

            if (!Double.isNaN(startPos)) {
                turnRelEncoder.setPosition(startPos);
                isTurretHomed = true;
            }
        }
        Logger.recordOutput("Turret/absoluteTurretAngle", calculateAbsoluteTurretAngle());

        double targetHoodEncoderAngle = (turretHoodAngle + hoodAngleOffset);
        hoodController.setSetpoint(targetHoodEncoderAngle, ControlType.kPosition);

        // Gate turret motion until absolute homing completes.
        if (isTurretHomed) {
            turretController.setSetpoint(turretAngleSetpoint, ControlType.kMAXMotionPositionControl);
        } else {
            turretController.setSetpoint(0.0, ControlType.kVoltage);
        }

        turretAngle = getTurretPos();
        inputs.turretAngle = turretAngle;
        inputs.turretSpeed = turretSpeed;
        inputs.turretHoodAngle = turretHoodAngle;
        inputs.turretAngleSetpoint = turretAngleSetpoint;
        inputs.turretEnc1Pos = turnEncoder1.get();
        inputs.turretEnc2Pos = turnEncoder2.get();

        sparkStickyFault = false;
        ifOk(
                turretSpark,
                turnRelEncoder::getPosition,
                (value) -> inputs.turretPosition = value);
        ifOk(turretSpark, turnRelEncoder::getVelocity, (value) -> inputs.turretVelocity = value);
        ifOk(
                turretSpark,
                new DoubleSupplier[] { turretSpark::getAppliedOutput, turretSpark::getBusVoltage },
                (values) -> inputs.turretAppliedVolts = values[0] * values[1]);
        ifOk(turretSpark, turretSpark::getOutputCurrent, (value) -> inputs.turretCurrentAmps = value);
        inputs.turretConnected = turretConnectedDebounce.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        ifOk(
                hoodSpark,
                () -> getHoodAngle(),
                (value) -> inputs.hoodPositionDeg = value);
        ifOk(hoodSpark, hoodEncoder::getVelocity, (value) -> inputs.hoodVelocityDegPerSec = value);
        ifOk(
                hoodSpark,
                new DoubleSupplier[] { hoodSpark::getAppliedOutput, hoodSpark::getBusVoltage },
                (values) -> inputs.hoodAppliedVolts = values[0] * values[1]);
        ifOk(hoodSpark, hoodSpark::getOutputCurrent, (value) -> inputs.hoodCurrentAmps = value);
        inputs.hoodConnected = hoodConnectedDebounce.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        ifOk(
                flywheel1Spark,
                flywheel1Encoder::getPosition,
                (value) -> inputs.flywheelPositionMeters = value);
        ifOk(flywheel1Spark, flywheel1Encoder::getVelocity, (value) -> inputs.flywheelVelocity = value);
        ifOk(flywheel2Spark, flywheel2Encoder::getVelocity, (value) -> inputs.flywheelFollowerVelocity = value);
        ifOk(
                flywheel1Spark,
                new DoubleSupplier[] { flywheel1Spark::getAppliedOutput, flywheel1Spark::getBusVoltage },
                (values) -> inputs.flywheelAppliedVolts = values[0] * values[1]);
        ifOk(flywheel1Spark, flywheel1Spark::getOutputCurrent, (value) -> inputs.flywheelCurrentAmps = value);
        ifOk(flywheel2Spark, flywheel2Spark::getOutputCurrent, (value) -> inputs.flywheelFollowerCurrentAmps = value);
        inputs.flywheelConnected = flywheelConnectedDebounce.calculate(!sparkStickyFault);
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double corrected = MathUtil.inputModulus(rotation.getDegrees(), -120, 240);
        turretAngleSetpoint = MathUtil.clamp(corrected, -120, 240); // -30 329
    }

    @Override
    public void setTurnVoltage(double volts) {
        turretSpark.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setHoodAngle(double angle) {
        turretHoodAngle = angle;
    };

    @Override
    public double getHoodAngle() {
        return hoodEncoder.getPosition() - hoodAngleOffset;
    }

    @Override
    public void setShotAngle(double angle) {
        setHoodAngle(MathUtil.clamp(angle, 0.0, 78.5));
    }

    @Override
    public void setShooterSpeed(double speed) {
        turretSpeed = speed;
        shooterVoltageCommand = 0.0;
        flywheelController1.setSetpoint(turretSpeed, ControlType.kVelocity);
        flywheelController2.setSetpoint(turretSpeed, ControlType.kVelocity);
    };

    @Override
    public void setShooterVoltage(double volts) {
        shooterVoltageCommand = MathUtil.clamp(volts, -12.0, 12.0);
        flywheel1Spark.setVoltage(shooterVoltageCommand);
        flywheel2Spark.setVoltage(shooterVoltageCommand);
    }

    @Override
    public boolean flywheelAtSpeed() {
        return flywheel1Encoder.getVelocity() > (turretSpeed * flywheelReadyRatio);
    }

    @Override
    public void shootFuel() {

        // Code for making turret shoot fuel to position?

    }

    public void setTurretHomed(boolean homed) {
        this.isTurretHomed = homed;
    }
}
