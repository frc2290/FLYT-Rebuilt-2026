// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.StateMachines.StateMachine;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SitAndShoot extends FlytSequentialAuto {
    private StateMachine stateMachine;

    /** Creates a new SitAndShoot. */
    public SitAndShoot(StateMachine stateMachine) {
        this.stateMachine = stateMachine;
    }

    @Override
    public void setup() {
        addCommands(this.stateMachine.setShooterOverrideCommand(true));
    }
}
