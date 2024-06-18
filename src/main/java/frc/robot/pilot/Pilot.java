package frc.robot.pilot;

import frc.robot.Robot;
import frc.robot.RobotCommands;
import frc.robot.RobotTelemetry;
import frc.robot.leds.LEDsCommands;
import frc.robot.mechanisms.elevator.ElevatorCommands;
import frc.robot.mechanisms.launcher.LauncherCommands;
import frc.robot.swerve.commands.SwerveCommands;
import frc.robot.vision.VisionCommands;
import frc.spectrumLib.gamepads.Gamepad;
import frc.spectrumLib.util.ExpCurve;

public class Pilot extends Gamepad {
    public class PilotConfig {
        public static final String name = "Pilot";
        public static final int port = 0;
        /**
         * in order to run a PS5 controller, you must use DS4Windows to emulate a XBOX controller as
         * well and move the controller to emulatedPS5Port
         */
        public static final boolean isXbox = true;

        public static final int emulatedPS5Port = 4;

        public static final double slowModeScalor = 0.45;
        public static final double turboModeScalor = 1;

        public final double leftStickDeadzone = 0; // TODO: reivew
        public final double leftStickExp = 2.0;
        public final double leftStickScalor = Robot.swerve.config.maxVelocity;

        public final double triggersDeadzone = 0; // TODO: review
        public final double triggersExp = 2.0;
        public final double triggersScalor = Robot.swerve.config.maxAngularVelocity;
        public final double rotationScalor = 0.8; // original was 0.8
    }

    public PilotConfig config;
    private boolean isSlowMode = false;
    private boolean isTurboMode = false;
    private boolean isFieldOriented = true;
    private ExpCurve LeftStickCurve;
    private ExpCurve TriggersCurve;

    /** Create a new Pilot with the default name and port. */
    public Pilot() {
        super(PilotConfig.name, PilotConfig.port, PilotConfig.isXbox, PilotConfig.emulatedPS5Port);
        config = new PilotConfig();

        // Curve objects that we use to configure the controller axis ojbects
        LeftStickCurve =
                new ExpCurve(
                        config.leftStickExp, 0, config.leftStickScalor, config.leftStickDeadzone);
        TriggersCurve =
                new ExpCurve(config.triggersExp, 0, config.triggersScalor, config.triggersDeadzone);

        RobotTelemetry.print("Pilot Subsystem Initialized: ");
    }

    /** Setup the Buttons for telop mode. */
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simualation */
    public void setupTeleopButtons() {

        controller
                .a()
                .and(noBumpers())
                .and(controller.x().negate())
                .whileTrue(RobotCommands.intake());
        controller.a().and(leftBumperOnly()).whileTrue(RobotCommands.eject());

        controller.b().and(noBumpers().or(rightBumperOnly())).whileTrue(PilotCommands.turnToAmp());
        controller
                .b()
                .and(noBumpers())
                .or(rightBumperOnly())
                .whileTrue(LauncherCommands.runAmpVelocity());

        controller
                .b()
                .and(leftBumperOnly().or(bothBumpers()))
                .whileTrue(RobotCommands.intoAmpShot());

        controller
                .x()
                .and(noBumpers().or(rightBumperOnly()))
                .and(controller.a().negate())
                .whileTrue(RobotCommands.visionLaunch());
        controller
                .x()
                .and(leftBumperOnly().or(bothBumpers()))
                .whileTrue(RobotCommands.manualFeedLaunch());

        // controller.start().onTrue(RobotCommands.climb()); // change pivot angle to max for climb

        // y - amp ready, and home
        /*controller
        .y()
        .and(noBumpers().or(rightBumperOnly()))
        .whileTrue(VisionCommands.driveToNote().alongWith(RobotCommands.smartIntake()));
        */
        controller
                .y()
                .and(leftBumperOnly().or(bothBumpers()))
                .whileTrue(RobotCommands.subwooferShot());

        controller
                .a()
                .and(controller.x())
                .and(noBumpers())
                .whileTrue(RobotCommands.instantFeedLaunch());

        controller.start().whileTrue(RobotCommands.autoClimb());
        controller.select().whileTrue(SwerveCommands.cardinalReorient());

        runWithEndSequence(rightBumperOnly(), RobotCommands.score(), ElevatorCommands.home());
        controller
                .leftBumper()
                .and(controller.rightBumper())
                .whileTrue(RobotCommands.launchEject());

        controller.rightStick().whileTrue(PilotCommands.slowMode());

        rightStick().and(leftBumperOnly()).whileTrue(PilotCommands.manualPivot());

        controller.leftDpad().and(noBumpers()).whileTrue(RobotCommands.manualSource());
        controller.rightDpad().and(noBumpers()).whileTrue(RobotCommands.centerClimbAlign());

        controller.leftDpad().and(noBumpers()).whileTrue(RobotCommands.manualSource());

        controller
                .upDpad()
                .and(leftBumperOnly())
                .whileTrue(rumbleCommand(SwerveCommands.reorientForward()));
        controller
                .leftDpad()
                .and(leftBumperOnly())
                .whileTrue(rumbleCommand(SwerveCommands.reorientLeft()));
        controller
                .downDpad()
                .and(leftBumperOnly())
                .whileTrue(rumbleCommand(SwerveCommands.reorientBack()));
        controller
                .rightDpad()
                .and(leftBumperOnly())
                .whileTrue(rumbleCommand(SwerveCommands.reorientRight()));

        // Use the pilot drive if we are manually steering the robot
        controller
                .rightTrigger(config.triggersDeadzone)
                .or(controller.leftTrigger(config.triggersDeadzone))
                .whileTrue(PilotCommands.pilotDrive());

        // Use the right stick to set a cardinal direction to aim at
        (leftBumperOnly().negate())
                .and(
                        rightXTrigger(ThresholdType.ABS_GREATER_THAN, 0.5)
                                .or(rightYTrigger(ThresholdType.ABS_GREATER_THAN, 0.5)))
                .whileTrue(PilotCommands.stickSteerDrive());
    };

    /** Setup the Buttons for Disabled mode. */
    public void setupDisabledButtons() {
        // This is just for training, most robots will have different buttons during disabled
        // setupTeleopButtons();
        controller.a().whileTrue(LEDsCommands.solidWhiteLED());
        controller.y().whileTrue(PilotCommands.rumble(1, 0.5).ignoringDisable(true));

        controller.b().toggleOnTrue(RobotCommands.coastModeMechanisms());
        controller.a().whileTrue(VisionCommands.forcePoseToVision());
    };

    /** Setup the Buttons for Test mode. */
    public void setupTestButtons() {
        // This is just for training, robots may have different buttons during test
        setupTeleopButtons();
    };

    public void setMaxVelocity(double maxVelocity) {
        LeftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        TriggersCurve.setScalar(maxRotationalVelocity);
    }

    public void setSlowMode(boolean isSlowMode) {
        this.isSlowMode = isSlowMode;
    }

    public void setTurboMode(boolean isTurboMode) {
        this.isTurboMode = isTurboMode;
    }

    public void setFieldOriented(boolean isFieldOriented) {
        this.isFieldOriented = isFieldOriented;
    }

    public boolean getFieldOriented() {
        return isFieldOriented;
    }

    // Positive is forward, up on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = LeftStickCurve.calculate(-1 * controller.getLeftY());
        // if (isSlowMode) {
        //     fwdPositive *= Math.abs(PilotConfig.slowModeScalor);
        // }
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = -1 * LeftStickCurve.calculate(controller.getLeftX());
        // if (isSlowMode) {
        //     leftPositive *= Math.abs(PilotConfig.slowModeScalor);
        // }
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveCCWPositive() {
        double ccwPositive = TriggersCurve.calculate(getTwist());
        if (isSlowMode) {
            ccwPositive *= Math.abs(PilotConfig.slowModeScalor);
        } else if (isTurboMode) {
            ccwPositive *= Math.abs(PilotConfig.turboModeScalor);
        } else {
            ccwPositive *= config.rotationScalor;
        }
        return ccwPositive;
    }
}
