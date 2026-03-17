package frc.robot.subsystems.dyerotor;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.subsystems.dyerotor.DyeRotorConstants.*;
import static frc.utils.SparkUtil.ifOk;

import com.revrobotics.REVLibError;
import java.util.function.DoubleSupplier;
import edu.wpi.first.math.MathUtil;

public class DyeRotorIOSpark implements DyeRotorIO {
    // Harware objects
    private final SparkBase feeder;
    private final SparkBase rotor;
    private RelativeEncoder feederEnc;
    private RelativeEncoder rotorEnc;

    // Closed loop controllers
    private final SparkClosedLoopController feederController;
    private final SparkClosedLoopController rotorController;

    public DyeRotorIOSpark() {

        // Create motor controllers
        feeder = new SparkMax(feederCanId, MotorType.kBrushless);
        rotor = new SparkFlex(rotorCanId, MotorType.kBrushless);

        // Setup encoders
        feederEnc = feeder.getEncoder();
        rotorEnc = rotor.getEncoder();

        // Set up Controllers
        feederController = feeder.getClosedLoopController();
        rotorController = rotor.getClosedLoopController();

        var rotorConfig = new SparkFlexConfig();
        rotorConfig
                .inverted(rotorIsInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(rotorMotorCurrent)
                .voltageCompensation(12.0);
        rotorConfig
                .encoder
                .positionConversionFactor(rotorEncoderPositionFactor)
                .velocityConversionFactor(rotorEncoderVelocityFactor)
                .quadratureAverageDepth(2);
        rotorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(rotorKp, rotorKi, rotorKd)
                .allowedClosedLoopError(rotorReadyToleranceRps, ClosedLoopSlot.kSlot0);
        rotorConfig.closedLoop.feedForward.kV(rotorKv);
        REVLibError rotorErr = rotor.configure(
                rotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        if (rotorErr != REVLibError.kOk) {
                System.err.println("DYEROTOR ROTOR CONFIG FAILED: " + rotorErr.name());
        }

        var feederConfig = new SparkMaxConfig();
        feederConfig
                .inverted(feederIsInverted)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(feederMotorCurrent)
                .voltageCompensation(12.0);
        feederConfig
                .encoder
                .positionConversionFactor(feederEncoderPositionFactor)
                .velocityConversionFactor(feederEncoderVelocityFactor)
                .uvwAverageDepth(2);
        feederConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(feederKp, feederKi, feederKd)
                .allowedClosedLoopError(feederReadyToleranceRps, ClosedLoopSlot.kSlot0);
        feederConfig.closedLoop.feedForward.kV(feederKv);
        REVLibError feederErr = feeder.configure(
                feederConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
        if (feederErr != REVLibError.kOk) {
                System.err.println("DYEROTOR FEEDER CONFIG FAILED: " + feederErr.name());
        }
    }

    // private double getFeederSpeed() {
    // return feederEnc.getVelocity();
    // }

    // private double getRotorSpeed() {
    // return rotorEnc.getVelocity();
    // }

    @Override
    public void updateInputs(DyeRotorIOInputs inputs) {
        inputs.rotorEncoderPosition = rotorEnc.getPosition();
        inputs.rotorEncoderRPM = rotorEnc.getVelocity();
        ifOk(
                rotor,
                new DoubleSupplier[] { rotor::getAppliedOutput, rotor::getBusVoltage },
                (values) -> inputs.rotorAppliedVolts = values[0] * values[1]);
        ifOk(rotor, rotor::getOutputCurrent, (value) -> inputs.rotorCurrentAmps = value);

        inputs.feederEncoderPosition = feederEnc.getPosition();
        inputs.feederEncoderRPM = feederEnc.getVelocity();
        ifOk(
                feeder,
                new DoubleSupplier[] { feeder::getAppliedOutput, feeder::getBusVoltage },
                (values) -> inputs.feederAppliedVolts = values[0] * values[1]);
        ifOk(feeder, feeder::getOutputCurrent, (value) -> inputs.feederCurrentAmps = value);
    }

    @Override
    public void setRotorSpeed(double speed) {
        rotorController.setSetpoint(speed, ControlType.kVelocity);
    }

    @Override
    public void setFeederSpeed(double speed) {
        feederController.setSetpoint(speed, ControlType.kVelocity);
    }

    @Override
    public void setRotorVoltage(double volts) {
        rotor.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void setFeederVoltage(double volts) {
        feeder.setVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void resetIntegrator() {
        rotorController.setIAccum(0.0);
        feederController.setIAccum(0.0);
    }

    @Override
    public boolean rotorAtSetpoint() {
        return rotorController.isAtSetpoint();
    }

    @Override
    public boolean feederAtSetpoint() {
        return feederController.isAtSetpoint();
    }
}
