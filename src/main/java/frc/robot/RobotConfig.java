package frc.robot;

import edu.wpi.first.wpilibj.Timer;

public final class RobotConfig {

    public static String MAC = "00:80:2F:38:D9:B6"; // TODO: change; MAC stalling
    public static final Double robotInitDelay = 2.0; // Seconds to wait before starting robot code
    public final String ULTRAVIOLET2024MAC = "00:80:2F:38:D9:B6";
    public final String NOTEBLOCK2023MAC = "00:80:2F:19:0D:CE";
    public final String MUSICDISC2023MAC = "00:80:2F:23:E9:33";
    public final String XRAY2023MAC = "00:80:2F:22:50:6D";

    public final String Canivore = "3847";

    public static final int ledPWMport = 0;

    private RobotType robotType = null;
    public boolean intakeAttached = true;
    public boolean ampTrapAttached = true;
    public boolean elevatorAttached = true;
    public boolean feederAttached = true;

    public RobotConfig() {
        if (Robot.isReal()) {
            Timer.delay(RobotConfig.robotInitDelay); // Wait for the robot to fully boot up
        }
        // Set the MAC Address for this robot, useful for adjusting comp/practice bot
        // settings
        // MAC = Network.getMACaddress();
        // RobotTelemetry.print("Robot MAC: " + MAC);

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
                break;
            case XRAY:
                intakeAttached = false;
                ampTrapAttached = false;
                elevatorAttached = true;
                feederAttached = false;
            default:
                /* Set all the constants specifically for the robot */
                break;
        }
    }

    /** Set the RobotType based on if simulation or the MAC address of the RIO */
    public RobotType checkRobotType() {
        if (Robot.isSimulation()) {
            robotType = RobotType.SIM;
            RobotTelemetry.print("Robot Type: Simulation");
        } else if (MAC.equals(ULTRAVIOLET2024MAC)) {
            robotType = RobotType.ULTRAVIOLET;
            RobotTelemetry.print("Robot Type: ULTRAVIOLET");
        } else if (MAC.equals(NOTEBLOCK2023MAC)) {
            robotType = RobotType.NOTEBLOCK;
            RobotTelemetry.print("Robot Type: NOTEBLOCK");
        } else if (MAC.equals(MUSICDISC2023MAC)) {
            robotType = RobotType.MUSICDISC;
            RobotTelemetry.print("Robot Type: MUSICDISC");
        } else {
            robotType = RobotType.XRAY;
            RobotTelemetry.print("Robot Type: XRAY");
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
