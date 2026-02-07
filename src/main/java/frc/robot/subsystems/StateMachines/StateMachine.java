package frc.robot.subsystems.StateMachines;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.DyeRotor;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeConstants.IntakeSide;
import frc.robot.subsystems.turret.Turret;
import frc.utils.FlytDashboard;

public class StateMachine extends SubsystemBase {
    public enum State {
        NOTHING, // nothing
        SHOOT_SCORE, // shoot into hub
        SHOOT_ZONE, // shoot into alliance zone
    }

    boolean intake = false;
    IntakeSide intakeSide = IntakeSide.LEFT;
    private State state = State.NOTHING;

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

    }

    /**
     * get current state
     * 
     * @return state
     */
    public State getState() {
        return state;
    }

    /**
     * this function, setState, takes in a State state and sets the
     * state to the State state that was inputted into setState
     * 
     * @param state state to set
     */
    public void setState(State state) {
        // sets the state to the state (State state) provided
        // by the State state argument to setState
        this.state = state;
    }
}