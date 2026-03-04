package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;
import static frc.utils.SparkUtil.*;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
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

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.utils.DualEncoderUnwrapper;

public class TurretIOSpark implements TurretIO {

    // Harware objects
    private final SparkBase turretSpark;
    private final SparkBase hoodSpark; 
    private final SparkBase flywheel1Spark; //later change to bottom/top
    private final SparkBase flywheel2Spark;

    // Fo turret
    private RelativeEncoder turnRelEncoder; // Experimental for using one encoder for two things at the same time
    private AbsoluteEncoder turnEncoder;
    private DutyCycleEncoder turnEncoder1;
    private DutyCycleEncoder turnEncoder2;
    //private AbsoluteEncoder turnEncoder2;
    private AbsoluteEncoder hoodEncoder;
    private RelativeEncoder flywheel1Encoder;
    private RelativeEncoder flywheel2Encoder;

    // Closed loop controllers
    private final SparkClosedLoopController turretController; 
    private final SparkClosedLoopController hoodController;
    private final SparkClosedLoopController flywheelController1;

    private final Debouncer turretConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer hoodConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer flywheelConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    //

    // Global variables
    private double turretAngle = 0;
    private double turretSpeed = 0;
    private double turretHoodAngle = 10;
    private double turretAngleSetpoint = 0;

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
        turnEncoder1 = new DutyCycleEncoder(0, 1, 0.2664760066619002);
        turnEncoder2 = new DutyCycleEncoder(1, 1, 0.2869740071743502);

        // Gets if the encoders are connected, IMPLEMENT LATER
        turnEncoder1.isConnected();
        turnEncoder2.isConnected();

        /*In 2025 the API changed to remove rollover detection as rollover 
        detection did not work. The get() method returns the value within a
        rotation where the maximum value in a rotation is defined in the constructor\
        (default 1).*/

        // Setup Controllers
        turretController = turretSpark.getClosedLoopController();
        hoodController = hoodSpark.getClosedLoopController();
        flywheelController1 = flywheel1Spark.getClosedLoopController();

        // Turret parameters
        var turretConfig = new SparkFlexConfig();
        turretConfig
            .inverted(turretIsInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(turretMotorCurrent);
        turretConfig
            .encoder
            .positionConversionFactor(turretEncoderPositionFactor)
            .velocityConversionFactor(turretEncoderVelocityFactor)
            .quadratureAverageDepth(2);
        turretConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(turretKp, turretKi, turretKd);
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
            .smartCurrentLimit(hoodMotorCurrent);
        hoodConfig
            .absoluteEncoder
            .inverted(hoodEncoderInverted)
            .zeroOffset(hoodEncoderZeroOffset)
            .positionConversionFactor(hoodEncoderPositionFactor)
            .velocityConversionFactor(hoodEncoderVelocityFactor)
            .averageDepth(2)
            .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoderV2);
        hoodConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(hoodKp, hoodKi, hoodKd);
        //hoodConfig.closedLoop.feedForward.kV(hoodKv);
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

        // Flywheel leader config
        var flywheelLeaderConfig = new SparkFlexConfig();
        flywheelLeaderConfig
            .apply(flywheelBaseConfig)
            .inverted(flywheelIsInverted);
        flywheelLeaderConfig
            .encoder
            .positionConversionFactor(flywheelEncoderPositionFactor)
            .velocityConversionFactor(flywheelEncoderVelocityFactor)
            .quadratureMeasurementPeriod(10)
            .quadratureAverageDepth(2);
        flywheelLeaderConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(flywheelKp, flywheelKi, flywheelKd);
        flywheelLeaderConfig.closedLoop.feedForward.kV(flywheelKv);

        // Flywheel follower config
        var flywheelFollowerConfig = new SparkFlexConfig();
        flywheelFollowerConfig
            .apply(flywheelBaseConfig)
            // true means follower output is inverted relative to leader
            .follow(flywheel1Spark, true);
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

        // this is very important line that checks turrets position through absalute encoders then sets relative encoder to same pose + whataever offset we want
        turnRelEncoder.setPosition(getTurretPosAtStart() + encoderOffset);
    }


    // Absolute dual encoder position of turret estimator master piece
    // Also I think this function should be run only once to set position for relative encoder
    private double getTurretPosAtStart() {
        // Ratios: Turn mechanism gear / encoder pulleys.
        // Limits: Assumes the turret starts within +/- rangeTurret/2 rotations of mechanism travel.
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
            return 0.0;
        }
    }


    /**
     * Get turret pose
     * @return
     */
    private double getTurretPos(){
        return turnRelEncoder.getPosition();
    }
    private double getTurretVel(){
        return turnRelEncoder.getVelocity();
    }

    // I think this is used for logger pro to keep track of things?
    @Override
    public void updateInputs(TurretIOInputs inputs) {
        double targetHoodEncoderAngle = (turretHoodAngle + hoodAngleOffset);
        hoodController.setSetpoint(targetHoodEncoderAngle, ControlType.kPosition);

        flywheelController1.setSetpoint(turretSpeed, ControlType.kVelocity);

        turretController.setSetpoint(turretAngleSetpoint, ControlType.kPosition);

        //turretAngle = getTurretPos();
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
            new DoubleSupplier[] {turretSpark::getAppliedOutput, turretSpark::getBusVoltage},
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
            new DoubleSupplier[] {hoodSpark::getAppliedOutput, hoodSpark::getBusVoltage},
            (values) -> inputs.hoodAppliedVolts = values[0] * values[1]);
        ifOk(hoodSpark, hoodSpark::getOutputCurrent, (value) -> inputs.hoodCurrentAmps = value);
        inputs.hoodConnected = hoodConnectedDebounce.calculate(!sparkStickyFault);

        sparkStickyFault = false;
        ifOk(
            flywheel1Spark,
            flywheel1Encoder::getPosition,
            (value) -> inputs.flywheelPositionMeters = value);
        ifOk(flywheel1Spark, flywheel1Encoder::getVelocity, (value) -> inputs.flywheelVelocity = value);
        ifOk(
            flywheel1Spark,
            new DoubleSupplier[] {flywheel1Spark::getAppliedOutput, flywheel1Spark::getBusVoltage},
            (values) -> inputs.flywheelAppliedVolts = values[0] * values[1]);
        ifOk(flywheel1Spark, flywheel1Spark::getOutputCurrent, (value) -> inputs.flywheelCurrentAmps = value);
        inputs.flywheelConnected = flywheelConnectedDebounce.calculate(!sparkStickyFault);
    }


    @Override
    public void setTurnPosition(Rotation2d rotation) {
        double targetRotation = rotation.getDegrees();
        if (targetRotation > 75) {
            targetRotation = 75;
        }
        if (targetRotation < -75) {
            targetRotation = -75;
        }
        turretAngleSetpoint = targetRotation;
    }


    @Override
    public void setHoodAngle(double angle) {
        turretHoodAngle = angle;
    };

    public double getHoodAngle() {
        return hoodEncoder.getPosition() - hoodAngleOffset;
    }

    @Override
    public void setShotAngle(double angle) {
        double targetHoodAngle = hoodShotAngleOffset - angle;
        setHoodAngle(targetHoodAngle);
    }

    @Override
    public void setShooterSpeed(double speed) {
        turretSpeed = speed;
    };

    @Override
    public boolean flywheelAtSpeed() {
        return flywheel1Encoder.getVelocity() > (turretSpeed * 0.8);
    }


    @Override
    public void shootFuel() {

        //Code for making turret shoot fuel to position?


    }

       
}
