package frc.spectrumLib;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.BuildConstants;
import org.littletonrobotics.junction.Logger;

public class Telemetry extends SubsystemBase {

    private static boolean disablePrints = false;

    public Telemetry() {
        super();
        /* Display the currently running commands on SmartDashboard*/
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncomitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }
        /* Display the currently running commands on SmartDashboard*/
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    /** Disable Print Statement */
    public static void disablePrints() {
        disablePrints = true;
    }

    /** Enable Print Statements */
    public static void enablePrints() {
        disablePrints = false;
    }

    /** Print a statment if they are enabled */
    public static void print(String output) {
        if (!disablePrints) {
            System.out.println(
                    "TIME: " + String.format("%.3f", Timer.getFPGATimestamp()) + " || " + output);
        }
    }
}
