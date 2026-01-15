package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.FlytDashboard;

public class IntakeXtakeSubsystem extends SubsystemBase {


    private SparkMax intakeXtakeMotor;
    private SparkFlex in_intakeMotor;

    private SparkFlexConfig in_intakeConfig = new SparkFlexConfig();
    private SparkMaxConfig intakeXtakeConfig = new SparkMaxConfig();

    private FlytDashboard dashboard = new FlytDashboard("IntakeXtakeSubsytem");

    private SparkClosedLoopController flywheel;
    private SparkClosedLoopController in_intake;

    private RelativeEncoder flywheelEnc;
    private RelativeEncoder in_intakeEnc;

    private double velocity_setpoint_flywheel;
    private double velocity_setpoint_in_intaek;

    public IntakeXtakeSubsystem() {

        in_intakeMotor = new SparkFlex(20, MotorType.kBrushless);
        intakeXtakeMotor = new SparkMax(21, MotorType.kBrushless);


        intakeXtakeConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);
        in_intakeConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(60);

        intakeXtakeMotor.configure(intakeXtakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        in_intakeMotor.configure(in_intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void intake(double speed) {
        intakeXtakeMotor.set(-speed);
        in_intakeMotor.set(speed);
    }

    public void shoot(double speed) {
        intakeXtakeMotor.set(speed);
        in_intakeMotor.set(speed);
    }

    public void hopperMotor(double speed) {
        intakeXtakeMotor.set(speed);
    }

    public void shooterMotor(double speed) {
        in_intakeMotor.set(speed);
    }
}
