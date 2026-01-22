package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeShooter extends SubsystemBase {
    private SparkFlex intakeShootMotor;
    private SparkMax redirectMotor;

    private SparkFlexConfig intakeShootConfig = new SparkFlexConfig();
    private SparkMaxConfig redirectConfig = new SparkMaxConfig();

    // add dashboard maybe?

    public IntakeShooter() {
        intakeShootMotor = new SparkFlex(20, MotorType.kBrushless);
        intakeShootConfig
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(60);

        redirectMotor = new SparkMax(21, MotorType.kBrushless);
        redirectConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);
        
        intakeShootMotor.configure(intakeShootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        redirectMotor.configure(redirectConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void intake(double speed) {
        intakeShootMotor.set(speed);
        redirectMotor.set(-speed);
    }

    public void shoot(double speed) {
        intakeShootMotor.set(speed);
        redirectMotor.set(speed);
    }

    public void shooterMotor(double speed) {
        intakeShootMotor.set(speed);
    }

    public void hopperMotor(double speed) {
        redirectMotor.set(speed);
    }
}
