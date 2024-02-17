package frc.robot.mechanisms.configs;

import frc.spectrumLib.mechanism.BaseMechConfig;
import frc.spectrumLib.mechanism.RobotMechConfig;

public class ALPHA2024 extends RobotMechConfig {

    public ALPHA2024() {
        super(
                AmpTrapBaseConfig,
                ClimberBaseConfig,
                ElevatorBaseConfig,
                FeederBaseConfig,
                IntakeBaseConfig,
                LeftLauncherBaseConfig,
                RightLauncherBaseConfig,
                PivotBaseConfig);
    }

    /* AmpTrap */
    private static class AmpTrap {
        private static String CANbus = "3847";
        private static int id = 51;
        private static double gearRatio = 12 / 30;
        private static boolean inverted = false;
        private static double startingPosition = 0;
    }

    /* Climber */
    private static class Climber {
        private static String CANbus = "3847";
        private static int id = 53;
        private static double gearRatio = 1;
        private static boolean inverted = true;
        private static double startingPosition = 0;
    }

    /* Elevator */
    private static class Elevator {
        private static String CANbus = "3847";
        private static int id = 52;
        private static double gearRatio = 1;
        private static boolean inverted = true;
        private static double startingPosition = 0;
    }

    /* Feeder */
    private static class Feeder {
        private static String CANbus = "3847";
        private static int id = 40;
        private static double gearRatio = 12 / 30;
        private static boolean inverted = false;
        private static double startingPosition = 0;
    }

    /* Intake */
    private static class Intake {
        private static String CANbus = "3847";
        private static int id = 60;
        private static double gearRatio = 12 / 30;
        private static boolean inverted = true;
        private static double startingPosition = 0;
    }

    /* Right Launcher */
    private static class RightLauncher {
        private static String CANbus = "3847";
        private static int id = 43;
        private static double gearRatio = 1 / 2;
        private static boolean inverted = true;
        private static double startingPosition = 0;
    }

    /* Left Launcher */
    private static class LeftLauncher {
        private static String CANbus = "3847";
        private static int id = 42;
        private static double gearRatio = 1 / 2;
        private static boolean inverted = false;
        private static double startingPosition = 0;
    }

    /* Pivot */
    private static class Pivot {
        private static String CANbus = "3847";
        private static int id = 41;
        private static double gearRatio = 1;
        private static boolean inverted = true;
        private static double startingPosition = 0;
    }

    public static final BaseMechConfig AmpTrapBaseConfig =
            new BaseMechConfig()
                    .withCANbusName(AmpTrap.CANbus)
                    .withId(AmpTrap.id)
                    .withGearRatio(AmpTrap.gearRatio)
                    .withInverted(AmpTrap.inverted)
                    .withStartingPos(AmpTrap.startingPosition);

    public static final BaseMechConfig ClimberBaseConfig =
            new BaseMechConfig()
                    .withCANbusName(Climber.CANbus)
                    .withId(Climber.id)
                    .withGearRatio(Climber.gearRatio)
                    .withInverted(Climber.inverted)
                    .withStartingPos(Climber.startingPosition);

    public static final BaseMechConfig ElevatorBaseConfig =
            new BaseMechConfig()
                    .withCANbusName(Elevator.CANbus)
                    .withId(Elevator.id)
                    .withGearRatio(Elevator.gearRatio)
                    .withInverted(Elevator.inverted)
                    .withStartingPos(Elevator.startingPosition);

    public static final BaseMechConfig FeederBaseConfig =
            new BaseMechConfig()
                    .withCANbusName(Feeder.CANbus)
                    .withId(Feeder.id)
                    .withGearRatio(Feeder.gearRatio)
                    .withInverted(Feeder.inverted)
                    .withStartingPos(Feeder.startingPosition);

    public static final BaseMechConfig IntakeBaseConfig =
            new BaseMechConfig()
                    .withCANbusName(Intake.CANbus)
                    .withId(Intake.id)
                    .withGearRatio(Intake.gearRatio)
                    .withInverted(Intake.inverted)
                    .withStartingPos(Intake.startingPosition);

    public static final BaseMechConfig RightLauncherBaseConfig =
            new BaseMechConfig()
                    .withCANbusName(RightLauncher.CANbus)
                    .withId(RightLauncher.id)
                    .withGearRatio(RightLauncher.gearRatio)
                    .withInverted(RightLauncher.inverted)
                    .withStartingPos(RightLauncher.startingPosition);

    public static final BaseMechConfig LeftLauncherBaseConfig =
            new BaseMechConfig()
                    .withCANbusName(LeftLauncher.CANbus)
                    .withId(LeftLauncher.id)
                    .withGearRatio(LeftLauncher.gearRatio)
                    .withInverted(LeftLauncher.inverted)
                    .withStartingPos(LeftLauncher.startingPosition);

    public static final BaseMechConfig PivotBaseConfig =
            new BaseMechConfig()
                    .withCANbusName(Pivot.CANbus)
                    .withId(Pivot.id)
                    .withGearRatio(Pivot.gearRatio)
                    .withInverted(Pivot.inverted)
                    .withStartingPos(Pivot.startingPosition);
}
