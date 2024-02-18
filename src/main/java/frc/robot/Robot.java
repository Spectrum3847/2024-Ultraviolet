package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auton.Auton;
import frc.robot.leds.LEDs;
import frc.robot.leds.LEDsCommands;
import frc.robot.mechanisms.amptrap.AmpTrap;
import frc.robot.mechanisms.amptrap.AmpTrapCommands;
import frc.robot.mechanisms.climber.Climber;
import frc.robot.mechanisms.climber.ClimberCommands;
import frc.robot.mechanisms.elevator.Elevator;
import frc.robot.mechanisms.elevator.ElevatorCommands;
import frc.robot.mechanisms.feeder.Feeder;
import frc.robot.mechanisms.feeder.FeederCommands;
import frc.robot.mechanisms.intake.Intake;
import frc.robot.mechanisms.intake.IntakeCommands;
import frc.robot.mechanisms.launcher.LauncherCommands;
import frc.robot.mechanisms.launcher.LeftLauncher;
import frc.robot.mechanisms.launcher.RightLauncher;
import frc.robot.mechanisms.pivot.Pivot;
import frc.robot.mechanisms.pivot.PivotCommands;
import frc.robot.operator.Operator;
import frc.robot.operator.OperatorCommands;
import frc.robot.pilot.Pilot;
import frc.robot.pilot.PilotCommands;
import frc.robot.swerve.Swerve;
import frc.robot.swerve.commands.SwerveCommands;
import frc.robot.vision.Vision;
import frc.spectrumLib.util.CrashTracker;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    public static RobotConfig config;
    public static RobotTelemetry telemetry;

    /** Create a single static instance of all of your subsystems */
    public static Swerve swerve;

    public static Intake intake;
    public static AmpTrap ampTrap;
    public static Elevator elevator;
    public static Feeder feeder;
    public static Climber climber;
    public static Pivot pivot;
    public static LeftLauncher leftLauncher;
    public static RightLauncher rightLauncher;
    public static Vision vision;
    public static Auton auton;
    public static LEDs leds;
    public static Pilot pilot;
    public static Operator operator;
    // public static Auton auton;

    /**
     * This method cancels all commands and returns subsystems to their default commands and the
     * gamepad configs are reset so that new bindings can be assigned based on mode This method
     * should be called when each mode is intialized
     */
    public static void resetCommandsAndButtons() {
        CommandScheduler.getInstance().cancelAll(); // Disable any currently running commands
        CommandScheduler.getInstance().getActiveButtonLoop().clear();

        // Reset Config for all gamepads and other button bindings
        pilot.resetConfig();
        operator.resetConfig();
    }

    /* ROBOT INIT (Initialization) */
    /** This method is called once when the robot is first powered on. */
    public void robotInit() {
        try {
            RobotTelemetry.print("--- Robot Init Starting ---");

            /* Start AdvantageKit */
            advantageKitInit();

            /** Set up the config */
            config = new RobotConfig();

            /**
             * Intialize the Subsystems of the robot. Subsystems are how we divide up the robot
             * code. Anything with an output that needs to be independently controlled is a
             * subsystem Something that don't have an output are alos subsystems.
             */
            swerve = new Swerve();
            intake = new Intake(config.intakeAttached);
            ampTrap = new AmpTrap(config.ampTrapAttached);
            elevator = new Elevator(config.elevatorAttached);
            feeder = new Feeder(config.feederAttached);
            climber = new Climber(config.climberAttached);
            pivot = new Pivot(config.pivotAttached);
            leftLauncher = new LeftLauncher(config.leftLauncherAttached);
            rightLauncher = new RightLauncher(config.rightLauncherAttached);
            vision = new Vision();
            auton = new Auton();
            pilot = new Pilot();
            operator = new Operator();
            leds = new LEDs();

            /** Intialize Telemetry and Auton */
            telemetry = new RobotTelemetry();
            // auton = new Auton();

            /**
             * Set Default Commands this method should exist for each subsystem that has default
             * command these must be done after all the subsystems are intialized
             */
            SwerveCommands.setupDefaultCommand();
            IntakeCommands.setupDefaultCommand();
            AmpTrapCommands.setupDefaultCommand();
            ElevatorCommands.setupDefaultCommand();
            FeederCommands.setupDefaultCommand();
            PivotCommands.setupDefaultCommand();
            LauncherCommands.setupDefaultCommand();
            LEDsCommands.setupDefaultCommand();
            PilotCommands.setupDefaultCommand();
            OperatorCommands.setupDefaultCommand();
            ClimberCommands.setupDefaultCommand();

            RobotTelemetry.print("--- Robot Init Complete ---");

        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /* ROBOT PERIODIC  */
    /**
     * This method is called periodically the entire time the robot is running. Periodic methods are
     * called every 20 ms (50 times per second) by default Since the robot software is always
     * looping you shouldn't pause the execution of the robot code This ensures that new values are
     * updated from the gamepads and sent to the motors
     */
    public void robotPeriodic() {
        try {
            /**
             * Runs the Scheduler. This is responsible for polling buttons, adding newly-scheduled
             * commands, running already-scheduled commands, removing finished or interrupted
             * commands, and running subsystem periodic() methods. This must be called from the
             * robot's periodic block in order for anything in the Command-based framework to work.
             */
            CommandScheduler.getInstance().run();
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /* DISABLED MODE */
    /** This mode is run when the robot is disabled All motor/accuator outputs are turned off */

    /** This method is called once when disabled starts */
    public void disabledInit() {
        RobotTelemetry.print("### Disabled Init Starting ### ");

        resetCommandsAndButtons();

        RobotTelemetry.print("### Disabled Init Complete ### ");
    }

    /** This method is called periodically while disabled. */
    public void disabledPeriodic() {}

    /** This method is called once when disabled exits */
    public void disabledExit() {
        RobotTelemetry.print("### Disabled Exit### ");
    }

    /* AUTONOMOUS MODE (AUTO) */
    /**
     * This mode is run when the DriverStation Software is set to autonomous and enabled. In this
     * mode the robot is not able to read values from the gamepads
     */

    /** This method is called once when autonomous starts */
    public void autonomousInit() {
        try {
            RobotTelemetry.print("@@@ Auton Init Starting @@@ ");
            resetCommandsAndButtons();
            Command autonCommand = Auton.getAutonomousCommand();

            if (autonCommand != null) {
                autonCommand.schedule();
                Auton.startAutonTimer();
            } else {
                RobotTelemetry.print("No Auton Command Found");
            }

            RobotTelemetry.print("@@@ Auton Init Complete @@@ ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /** This method is called periodically during autonomous. */
    public void autonomousPeriodic() {}

    /** This method is called once when autonomous exits */
    public void autonomousExit() {
        RobotTelemetry.print("@@@ Auton Exit @@@ ");
    }

    /* TELEOP MODE */
    /**
     * This mode is run when the DriverStation Software is set to teleop and enabled. In this mode
     * the robot is fully enabled and can move it's outputs and read values from the gamepads
     */

    /** This method is called once when teleop starts */
    public void teleopInit() {
        try {
            RobotTelemetry.print("!!! Teleop Init Starting !!! ");
            resetCommandsAndButtons();

            // flip pilot's forward based on what alliance robot is
            // swerve.setDriverPerspective(
            //         Rotation2d.fromDegrees(
            //                 DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue
            //                         ? 0
            //                         : 180));

            RobotTelemetry.print("!!! Teleop Init Complete !!! ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /** This method is called periodically during operator control. */
    public void teleopPeriodic() {}

    /** This method is called once when teleop exits */
    public void teleopExit() {
        RobotTelemetry.print("!!! Teleop Exit !!! ");
    }

    /* TEST MODE */
    /**
     * This mode is run when the DriverStation Software is set to test and enabled. In this mode the
     * is fully enabled and can move it's outputs and read values from the gamepads. This mode is
     * never enabled by the competition field It can be used to test specific features or modes of
     * the robot
     */

    /** This method is called once when test mode starts */
    public void testInit() {
        try {
            RobotTelemetry.print("~~~ Test Init Starting ~~~ ");
            resetCommandsAndButtons();

            RobotTelemetry.print("~~~ Test Init Complete ~~~ ");
        } catch (Throwable t) {
            // intercept error and log it
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    /** This method is called periodically during test. */
    public void testPeriodic() {}

    /** This method is called once when the robot exits test mode */
    public void testExit() {
        RobotTelemetry.print("~~~ Test Exit ~~~ ");
    }

    /* SIMULATION MODE */
    /**
     * This mode is run when the software is running in simulation and not on an actual robot. This
     * mode is never enabled by the competition field
     */

    /** This method is called once when a simulation starts */
    public void simulationInit() {
        RobotTelemetry.print("$$$ Simulation Init Starting $$$ ");

        RobotTelemetry.print("$$$ Simulation Init Complete $$$ ");
    }

    /** This method is called periodically during simulation. */
    public void simulationPeriodic() {}

    /** This method is called once at the end of RobotInit to begin logging */
    public void advantageKitInit() {
        /* Set up data receivers & replay source */
        Logger.addDataReceiver(new NT4Publisher()); // Running a physics simulator, log to NT

        if (!Robot.isSimulation()) {
            Logger.addDataReceiver(
                    new WPILOGWriter("/U")); // Running on a real robot, log to a USB stick
        }

        // Start AdvantageKit logger
        Logger.start();
    }
}
