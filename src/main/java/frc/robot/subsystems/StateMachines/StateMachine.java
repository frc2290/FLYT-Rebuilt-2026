package frc.robot.subsystems.StateMachines;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.robot.subsystems.turret.Turret;
import frc.utils.FlytDashboard;

public class StateMachine extends SubsystemBase {
    public enum State {
        NOTHING,     // nothing
        SHOOT_SCORE, // shoot into hub
        SHOOT_ZONE,  // shoot into alliance zone
    }

    boolean intake = false;
    IntakeSide intakeSide = IntakeSide.LEFT;
    private State state = State.NOTHING;

    private Intake m_intake;
    private Turret m_turret;
    
    public StateMachine(Intake intake, Turret turret) {
        m_intake = intake;
        m_turret = turret;
    }

    @Override
    public void periodic() {

    }
}