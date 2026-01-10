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
                
            }
            case -> SHOT {

            }
            case -> AMP {

            }
            case -> PRE_SHOT {
                
            }
            case -> PRE_AMP {

            }
            case -> INTAKE {

            }
            case -> EJECT {

            }
            default -> {

            }
        }
    }

}