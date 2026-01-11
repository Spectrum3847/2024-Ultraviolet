package main.java.frc.robot;

import frc.robot.mechanisms.amptrap;
import frc.robot.mechanisms.elevator;
import frc.robot.mechanisms.feeder;
import frc.robot.mechanisms.intake;
import frc.robot.mechanisms.launcher;
import frc.robot.mechanisms.pivot;

public class Coordinator {
    
    public void update() {}

    public void applyRobotState(State state) {
        switch (state) {
            case -> REHOME{
                AmpTrapCommands.home();
                ElevatorCommands.home();
                FeederCommands.home();
                IntakeCommands.home();
                LauncherCommands.home();
                PivotCommands.home();
            }
            case -> SHOT {
                LauncherCommands.score();
            }
            case -> AMP {
                
            }
            case -> PRE_SHOT {
                
            }
            case -> PRE_AMP {

            }
            case -> INTAKE {
                IntakeCommands.intake();

                AmpTrapCommands.home();
                ElevatorCommands.home();
                FeederCommands.home();
                LauncherCommands.home();
                PivotCommands.home();
            }
            case -> EJECT {
                IntakeCommands.ejectFromIntake();

                AmpTrapCommands.home();
                ElevatorCommands.home();
                FeederCommands.home();
                LauncherCommands.home();
                PivotCommands.home();
            }
            default -> {

            }
        }
    }

}