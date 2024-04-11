package frc.robot.auton;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
        // autonChooser.addOption("Source 4 Auto", new PathPlannerAuto("Source 4")); // Runs full
        // Auto
        // autonChooser.addOption("Front 6.5 Test", AutoPaths.Front6Point5()); // Runs full Auto
        // autonChooser.addOption("Front Alt 6 Test", AutoPaths.FrontAlt6()); // Runs full Auto
        // autonChooser.addOption("Source 4 Test", AutoPaths.Source4()); // Runs full Auto
        // autonChooser.addOption("Front 6", new PathPlannerAuto("Front 6")); // Runs full Auto

        // autonChooser.addOption("Test", AutoPaths.Test()); // Runs full Auto

        autonChooser.addOption("Subwoofer", new PathPlannerAuto("Subwoofer")); // Runs full Auto
        autonChooser.addOption("Test Auto", AutoPaths.Test()); // Runs full Auto

        SmartDashboard.putData("Auto Chooser", autonChooser);
    }

    // Setup the named commands
    public static void setupNamedCommands() {
        // Register Named Commands
        NamedCommands.registerCommand("alignToSpeaker", AutonCommands.trackSpeaker());
        NamedCommands.registerCommand("alignToNote", AutonCommands.trackNote());
        NamedCommands.registerCommand("launchReadySubwoofer", AutonCommands.launchReadySubwoofer());
        NamedCommands.registerCommand("launchReadyPreload", AutonCommands.launchReadyPreload());
        NamedCommands.registerCommand("launchReady1", AutonCommands.launchReady1());
        NamedCommands.registerCommand("launchReady2", AutonCommands.launchReady2());
        NamedCommands.registerCommand("launchReady3", AutonCommands.launchReady3());
        NamedCommands.registerCommand("launchReady4", AutonCommands.launchReady4());
        NamedCommands.registerCommand("launchReady5", AutonCommands.launchReady5());
        NamedCommands.registerCommand("launchReady6", AutonCommands.launchReady6());
        NamedCommands.registerCommand("launchReady7", AutonCommands.launchReady7());
        NamedCommands.registerCommand("launchReady8", AutonCommands.launchReady8());
        NamedCommands.registerCommand("launchReady9", AutonCommands.launchReady9());
        NamedCommands.registerCommand("visionLaunchReady", AutonCommands.visionLaunch());
        NamedCommands.registerCommand("launch", AutonCommands.launch());
        NamedCommands.registerCommand("launchShort", AutonCommands.launchShort());
        NamedCommands.registerCommand("smartIntake", AutonCommands.intake());
        NamedCommands.registerCommand("intakeCheck", AutonCommands.intakeCheck());
        NamedCommands.registerCommand("intakeFeed", AutonCommands.intakeFeed());

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
