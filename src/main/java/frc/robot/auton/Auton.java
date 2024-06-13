package frc.robot.auton;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.robot.auton.config.AutonConfig;
import frc.robot.mechanisms.amptrap.AmpTrapCommands;
import frc.robot.mechanisms.feeder.FeederCommands;
import frc.robot.mechanisms.intake.IntakeCommands;
import frc.robot.mechanisms.launcher.LauncherCommands;
import frc.robot.mechanisms.pivot.PivotCommands;

public class Auton extends SubsystemBase {
    public static final SendableChooser<Command> autonChooser = new SendableChooser<>();
    public static boolean trackNote = false;
    public static boolean trackSpeaker = false;
    public static boolean noteIntaked = false;
    public static boolean intakeCheck = false;
    private static boolean autoMessagePrinted = true;
    private static double autonStart = 0;

    // A chooser for autonomous commands
    public static void setupSelectors() {
        // Config Autos
        // autonChooser.addOption("1 Meter", new PathPlannerAuto("1 Meter Auto")); // Runs full Auto
        // autonChooser.addOption("3 Meter", new PathPlannerAuto("3 Meter Auto")); // Runs full Auto
        // autonChooser.addOption("5 Meter", new PathPlannerAuto("5 Meter Auto")); // Runs full Auto
        // autonChooser.addOption("Front 5", new PathPlannerAuto("Front 5")); // Runs full Auto

        // Competition Autos
        autonChooser.addOption(
                "Front 6.5 Auto", new PathPlannerAuto("Front 6.5")); // Runs full Auto
        autonChooser.addOption(
                "Front Alt 6 Auto", new PathPlannerAuto("Front Alt 6")); // Runs full Auto

        autonChooser.addOption("Front 43", new PathPlannerAuto("Front 43")); // Runs full Auto

        autonChooser.addOption(
                "Front Rush 12", new PathPlannerAuto("Front Rush 12")); // Runs full Auto

        autonChooser.addOption(
                "Front Rush 23", new PathPlannerAuto("Front Rush 23")); // Runs full Auto
        autonChooser.addOption(
                "Front Rush 13", new PathPlannerAuto("Front Rush 13")); // Runs full Auto
        autonChooser.addOption(
                "Front Rush 3", new PathPlannerAuto("Front Rush 3")); // Runs full Auto

        autonChooser.addOption(
                "Front Rush 23 Spit", new PathPlannerAuto("Front Rush 23 Spit")); // Runs full Auto
        autonChooser.addOption("Source 4 Auto", new PathPlannerAuto("Source 4")); // Runs full
        // Auto
        // autonChooser.addOption("Front 6.5 Test", AutoPaths.Front6Point5()); // Runs full Auto
        // autonChooser.addOption("Front Alt 6 Test", AutoPaths.FrontAlt6()); // Runs full Auto
        // autonChooser.addOption("Source 4 Test", AutoPaths.Source4()); // Runs full Auto
        // autonChooser.addOption("Front 6", new PathPlannerAuto("Front 6")); // Runs full Auto

        // autonChooser.addOption("Test", AutoPaths.Test()); // Runs full Auto

        autonChooser.addOption("Subwoofer", new PathPlannerAuto("Subwoofer")); // Runs full Auto

        // autonChooser.addOption("Madtown Test", new PathPlannerAuto("Madtown Test"));
        // autonChooser.addOption("Madtown Test 2", new PathPlannerAuto("Madtown Test 2"));
        // autonChooser.addOption("Madtown Test 3", new PathPlannerAuto("Madtown Test 3"));
        // autonChooser.addOption("Madtown Dect", AutoPaths.MadtownTest());

        // Madtown Autos
        autonChooser.addOption("Madtown Adaptive", AutoPaths.MadtownAdaptive());
        autonChooser.addOption("Madtown", new PathPlannerAuto("Madtown"));
        autonChooser.addOption("Madtown Alt", new PathPlannerAuto("Madtown Alt"));

        autonChooser.addOption("Do Nothing", Commands.print("Do Nothing Auto ran"));

        SmartDashboard.putData("Auto Chooser", autonChooser);
    }

    // Setup the named commands
    public static void setupNamedCommands() {
        // Register Named Commands
        NamedCommands.registerCommand("alignToSpeaker", AutonCommands.trackSpeaker());
        NamedCommands.registerCommand("alignToNote", AutonCommands.trackNote());

        NamedCommands.registerCommand("madtownRan", AutonCommands.madtownRan());
        NamedCommands.registerCommand("launchReadySubwoofer", AutonCommands.launchReadySubwoofer());
        NamedCommands.registerCommand("launchReadyPreload", AutonCommands.launchReadyPreload());

        NamedCommands.registerCommand("launchReadyPreload2", AutonCommands.launchReadyPreload2());
        NamedCommands.registerCommand("launchReady1", AutonCommands.launchReady1());
        NamedCommands.registerCommand("launchReady2", AutonCommands.launchReady2());
        NamedCommands.registerCommand("launchReady3", AutonCommands.launchReady3());
        NamedCommands.registerCommand("launchReady4", AutonCommands.launchReady4());
        NamedCommands.registerCommand("launchReady5", AutonCommands.launchReady5());
        NamedCommands.registerCommand("launchReady6", AutonCommands.launchReady6());
        NamedCommands.registerCommand("launchReady7", AutonCommands.launchReady7());
        NamedCommands.registerCommand("launchReady8", AutonCommands.launchReady8());
        NamedCommands.registerCommand("launchReady9", AutonCommands.launchReady9());

        NamedCommands.registerCommand("launchReady10", AutonCommands.launchReady10());
        NamedCommands.registerCommand("launchReady11", AutonCommands.launchReady11());

        NamedCommands.registerCommand("launchReady12", AutonCommands.launchReady12());

        NamedCommands.registerCommand("launchReady13", AutonCommands.launchReady13());

        NamedCommands.registerCommand("launchReady14", AutonCommands.launchReady14());

        NamedCommands.registerCommand("launchReady15", AutonCommands.launchReady15());

        NamedCommands.registerCommand("launchReady16", AutonCommands.launchReady16());
        NamedCommands.registerCommand("launchReady17", AutonCommands.launchReady17());

        NamedCommands.registerCommand("visionLaunchReady", AutonCommands.visionLaunch());
        NamedCommands.registerCommand("launch", AutonCommands.launch());
        NamedCommands.registerCommand("launchShort", AutonCommands.launchShort());
        NamedCommands.registerCommand("smartIntake", AutonCommands.intake());
        NamedCommands.registerCommand("intakeCheck", AutonCommands.intakeCheck());
        NamedCommands.registerCommand("intakeFeed", AutonCommands.intakeFeed());
        NamedCommands.registerCommand("spit", AutonCommands.spit());
        NamedCommands.registerCommand("spitReady", AutonCommands.spitReady());

        NamedCommands.registerCommand("spit2", AutonCommands.spit2());
        NamedCommands.registerCommand("spitReady2", AutonCommands.spitReady2());

        NamedCommands.registerCommand("spit3", AutonCommands.spit3());
        NamedCommands.registerCommand("spitReady3", AutonCommands.spitReady3());

        NamedCommands.registerCommand("spit4", AutonCommands.spit4());

        NamedCommands.registerCommand("resetPose", AutonCommands.resetPoseToVision());

        /* Stop Commands */
        NamedCommands.registerCommand("stopTracking", AutonCommands.stopTracking());
        NamedCommands.registerCommand("stopSmartIntake", AutonCommands.stopFeed());
        NamedCommands.registerCommand("stophIntakeCheck", AutonCommands.stopIntakeCheck());
        NamedCommands.registerCommand("stopIntake", IntakeCommands.stopMotor());
        NamedCommands.registerCommand("stopFeeder", FeederCommands.stopMotor());
        NamedCommands.registerCommand("stopAmpTrap", AmpTrapCommands.stopMotor());
        NamedCommands.registerCommand("stopLauncher", LauncherCommands.stopMotors());
        NamedCommands.registerCommand("stopPivot", PivotCommands.stopMotor());
    }

    // Subsystem Documentation:
    // https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html
    public Auton() {
        setupNamedCommands(); // registers named commands
        AutonConfig.configureAutoBuilder(); // configures the auto builder
        setupSelectors(); // runs the command to start the chooser for auto on shuffleboard

        RobotTelemetry.print("Auton Subsystem Initialized: ");
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public static Command getAutonomousCommand() {
        // return new CharacterizeLauncher(Robot.launcher);
        Command auton = autonChooser.getSelected(); // sees what auto is chosen on shuffleboard
        if (auton != null) {
            return auton; // checks to make sure there is an auto and if there is it runs an auto
        } else {
            return new PrintCommand(
                    "*** AUTON COMMAND IS NULL ***"); // runs if there is no auto chosen, which
            // shouldn't happen because of the default
            // auto set to nothing which still runs
            // something
        }
    }

    /** This method is called in AutonInit */
    public static void startAutonTimer() {
        autonStart = Timer.getFPGATimestamp();
        autoMessagePrinted = false;
    }

    /** Called in RobotPeriodic and displays the duration of the auton command Based on 6328 code */
    public static void printAutoDuration() {
        Command autoCommand = Auton.getAutonomousCommand();
        if (autoCommand != null) {
            if (!autoCommand.isScheduled() && !autoMessagePrinted) {
                if (DriverStation.isAutonomousEnabled()) {
                    RobotTelemetry.print(
                            String.format(
                                    "*** Auton finished in %.2f secs ***",
                                    Timer.getFPGATimestamp() - autonStart));
                } else {
                    RobotTelemetry.print(
                            String.format(
                                    "*** Auton CANCELLED in %.2f secs ***",
                                    Timer.getFPGATimestamp() - autonStart));
                }
                autoMessagePrinted = true;
            }
        }
    }
}
