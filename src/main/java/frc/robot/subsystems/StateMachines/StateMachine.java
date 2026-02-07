package frc.robot.subsystems.StateMachines;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.DyeRotor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.robot.subsystems.turret.Turret;

public class StateMachine extends SubsystemBase {
    public enum ShooterState {
        STOP,           // stop.
        SHOOT_SCORE,    // shoot into hub
        SHOOT_ALLIANCE, // shoot into alliance zone
        SHOOT_NEUTRAL,  // shoot into neutral zone
    }

    public enum FieldZone {
        ALLIANCE,      // alliance zone
        NEUTRAL,       // neutral zone
        ANTI_ALLIANCE, // opposite alliance zone
    }

    boolean intake = false;
    IntakeSide intakeSide = IntakeSide.LEFT;
    private ShooterState shooterState = ShooterState.STOP;
    private FieldZone fieldZone = FieldZone.ALLIANCE;

    private Intake m_intake;
    private Turret m_turret;
    private DyeRotor m_dyeRotor;

    public StateMachine(Intake intake, Turret turret, DyeRotor dyeRotor) {
        m_intake = intake;
        m_turret = turret;
        m_dyeRotor = dyeRotor;
    }

    @Override
    public void periodic() {
        if (fieldZone != FieldZone.ALLIANCE) {
            shooterState = ShooterState.STOP;
        }
    }

    public ShooterState getShooterState() {
        return shooterState;
    }

    public void setShooterState(ShooterState shooterState) {
        this.shooterState = shooterState;
    }

    public FieldZone getfieldZone() {
        return fieldZone;
    }

    public void setfieldZone(FieldZone fieldZone) {
        this.fieldZone = fieldZone;
    }
}