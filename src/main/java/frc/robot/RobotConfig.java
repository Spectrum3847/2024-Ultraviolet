package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public final class RobotConfig {

    public static String rioSerial = "empty";
    public static final Double robotInitDelay = 2.0; // Seconds to wait before starting robot code
    /* Deprecated */
    public final String ULTRAVIOLET2024MAC = "00:80:2F:38:D1:DA"; // This is Alpha MAC
    public final String NOTEBLOCK2023MAC = "00:80:2F:19:0D:CE";
    public final String MUSICDISC2023MAC = "00:80:2F:23:E9:33";
    public final String XRAY2023MAC = "00:80:2F:22:50:6D";

    public final String ULTRAVIOLET2024SERIAL = "032B1F69";
    public final String PM2024SERIAL = "03223839";
    public final String NOTEBLOCK2023SERIAL = ""; // TODO: find
    public final String MUSICDISC2023SERIAL = ""; // TODO: find
    public final String XRAY2023SERIAL = ""; // TODO: find

    public final String Canivore = "3847";

    public static final int ledPWMport = 0;

    private RobotType robotType = null;
    public boolean intakeAttached = true;
    public boolean ampTrapAttached = true;
    public boolean elevatorAttached = true;
    public boolean feederAttached = true;
    public boolean climberAttached = true;
    public boolean pivotAttached = true;
    public boolean leftLauncherAttached = true;
    public boolean rightLauncherAttached = true;

    public RobotConfig() {
        if (Robot.isReal()) {
            Timer.delay(RobotConfig.robotInitDelay); // Wait for the robot to fully boot up
        }
        // Set the RoboRio Serial number for this robot, useful for adjusting comp/practice bot
        // settings
        if (RobotController.getSerialNumber() != null) {
            rioSerial = RobotController.getSerialNumber();
            System.out.println("RIO SERIAL: " + rioSerial);
        }

        checkRobotType();
        switch (getRobotType()) {
            case SIM:
                /* Set all the constants specifically for the simulation*/
                break;
            case ULTRAVIOLET:
                intakeAttached = true;
                ampTrapAttached = true;
                elevatorAttached = true;
                feederAttached = true;
                climberAttached = true;
                pivotAttached = true;
                leftLauncherAttached = true;
                rightLauncherAttached = true;
                break;
            case XRAY:
                intakeAttached = false;
                ampTrapAttached = false;
                elevatorAttached = true;
                feederAttached = false;
                climberAttached = false;
                pivotAttached = false;
                leftLauncherAttached = false;
                rightLauncherAttached = false;
            default:
                /* Set all the constants specifically for the robot */
                break;
        }

        System.out.println("ROBOT: " + getRobotType());
    }

    /** Set the RobotType based on if simulation or the serial number of the RIO */
    public RobotType checkRobotType() {
        if (Robot.isSimulation()) {
            robotType = RobotType.SIM;
            RobotTelemetry.print("Robot Type: Simulation");
        } else if (rioSerial.equals(ULTRAVIOLET2024SERIAL)) {
            robotType = RobotType.ULTRAVIOLET;
            RobotTelemetry.print("Robot Type: ULTRAVIOLET");

        } else if (rioSerial.equals(PM2024SERIAL)) {
            robotType = RobotType.ULTRAVIOLET;
            RobotTelemetry.print("Robot Type: PM (Config set to ULTRAVIOLET)");
        } else if (rioSerial.equals(NOTEBLOCK2023SERIAL)) {
            robotType = RobotType.NOTEBLOCK;
            RobotTelemetry.print("Robot Type: NOTEBLOCK");
        } else if (rioSerial.equals(MUSICDISC2023SERIAL)) {
            robotType = RobotType.MUSICDISC;
            RobotTelemetry.print("Robot Type: MUSICDISC");
        } else {
            robotType = RobotType.ULTRAVIOLET;
            RobotTelemetry.print("Robot Type: ULTRAVIOLET");
        }
        return robotType;
    }

    public RobotType getRobotType() {
        return robotType;
    }

    public enum RobotType {
        ULTRAVIOLET,
        MUSICDISC,
        NOTEBLOCK,
        XRAY,
        SIM
    }
}
