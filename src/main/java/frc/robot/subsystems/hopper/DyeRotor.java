package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DyeRotor extends SubsystemBase {

    private final DyeRotorIO io;
    private final DyeRotorIOInputsAutoLogged inputs = new DyeRotorIOInputsAutoLogged();
    private boolean isRunning = false;

    public DyeRotor(DyeRotorIO io) {
        this.io = io;
    }

    /**
     * Activate Dye rotor
     * @param run
     */
    public void runDyeRotor(boolean run) {
        isRunning = run;
        io.runDyeRotor(run);
    }

    @Override
    public void periodic() {
        //No idea what to put here
    }
    
}
