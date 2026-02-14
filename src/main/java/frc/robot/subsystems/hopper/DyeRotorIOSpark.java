package frc.robot.subsystems.hopper;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.*;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;

import static frc.robot.subsystems.hopper.DyeRotorConstants.*;
import static frc.utils.SparkUtil.tryUntilOk;


public class DyeRotorIOSpark implements DyeRotorIO {



    // Harware objects
    private final SparkBase feeder;
    private final SparkBase rotor;
    private RelativeEncoder feederEnc;
    private RelativeEncoder rotorEnc;

    // Closed loop controllers
    private final SparkClosedLoopController feederController;
    private final SparkClosedLoopController rotorController;

    // Global variables
    boolean isRotorRunning = false;
    //...
    //...

    public DyeRotorIOSpark() {

        // Create motor controllers
        feeder = new SparkMax(feederCanId, MotorType.kBrushless);
        rotor = new SparkFlex(rotorCanId, MotorType.kBrushless);
        
        // Setup encoders
        feederEnc = feeder.getEncoder();
        rotorEnc = rotor.getEncoder();

        // Setup Controllers
        feederController = feeder.getClosedLoopController();
        rotorController = rotor.getClosedLoopController();

        var rotorConfig = new SparkMaxConfig();
        rotorConfig
            .inverted(rotorIsInverted)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(rotorMotorCurrent)
            .voltageCompensation(12.0);
        rotorConfig
            .encoder
            .positionConversionFactor(rotorEncoderPositionFactor)
            .velocityConversionFactor(rotorEncoderVelocityFactor)
            .uvwAverageDepth(2);
        rotorConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(rotorKp, rotorKi, rotorKd);
        tryUntilOk(feeder, 5, () -> feeder.configure(rotorConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters));

        var feederConfig = new SparkMaxConfig();
        feederConfig
            .inverted(feederIsInverted)
            .idleMode(IdleMode.kCoast)
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
            .pid(feederKp, feederKi, feederKd);
        tryUntilOk(feeder, 5, () -> feeder.configure(feederConfig, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters));



    }

    private double getFeederSpeed(){
        return feederEnc.getVelocity();
    }

    private double getRotorSpeed()
    {
        return rotorEnc.getVelocity();

    }



    // I think this is used for logger pro to keep track of things?
    @Override
    public void updateInputs(DyeRotorIOInputs inputs) {
        inputs.dyeRotorVel = getRotorSpeed();
        inputs.feedRate = getFeederSpeed();
        inputs.isRotorRunning = isRotorRunning;

    }

    @Override
    public void runDyeRotor(boolean run) {
        if (run) {
            setDyeRotorSpeed(def_rotorSpeed);
            setFeedRate(def_feedRate);
            isRotorRunning = true;
        } else {
            setDyeRotorSpeed(0);
            setFeedRate(0);
            isRotorRunning = false;
        }
    }

    @Override
    public void setDyeRotorSpeed(double speed) {
        feeder.set(speed);
        // feederController.setSetpoint(speed, ControlType.kVelocity);
    }
    @Override
    public void setFeedRate(double feedRate) {
        rotor.set(feedRate);
        // rotorController.setSetpoint(feedRate, ControlType.kVelocity);
    }
}