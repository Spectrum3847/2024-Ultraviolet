package frc.spectrumLib.vision;

import java.text.DecimalFormat;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.spectrumLib.vision.LimelightHelpers.LimelightResults;

public class Limelight {

    /* Limelight Configuration */

    /** Must connect to the name given in LL dashboard */
    private final String CAMERA_NAME;

    /* Debug */
    private final DecimalFormat df = new DecimalFormat();

    public Limelight(String cameraName) {
        this.CAMERA_NAME = cameraName;
    }

    public Limelight(String cameraName, int pipeline) {
        this(cameraName);
        setLimelightPipeline(pipeline);
    }

    /*
     *
     * Frequently Used Methods
     * 
     * 
     */

    /* ::: Basic Information Retrieval ::: */
    
    /**
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees / LL2: -29.8 to 29.8 degrees)
     */
    public double getHorizontalOffset() {
        return LimelightHelpers.getTX(CAMERA_NAME);
    }

    /**
     * @return Vertical Offset From Crosshair To Target in degrees (LL1: -20.5 degrees to 20.5 degrees / LL2: -24.85 to 24.85 degrees)
     */
    public double getVerticalOffset() {
        return LimelightHelpers.getTY(CAMERA_NAME);
    }

    /**
     * @return Whether the LL has any valid targets (apriltags or other vision targets) 
     */
    public boolean targetInView() {
        return LimelightHelpers.getTV(CAMERA_NAME);
    }

    /** @return whether the LL sees multiple tags or not */
    public boolean multipleTagsInView() {
        if(retrieveJSON() == null) return false;

        return retrieveJSON().targetingResults.targets_Fiducials.length > 1;
    }

    /** @return the tag ID of the apriltag most centered in the LL's view (or based on different criteria set in LL dasbhoard) */
    public double getClosestTagID() {
        return LimelightHelpers.getFiducialID(CAMERA_NAME);
    }




    /* ::: Pose Retrieval ::: */

    /**
     * @return the corresponding LL Pose3d for the alliance in DriverStation.java. Will default to blue if invalid alliance.
     */
    private Pose3d getAlliancePose() {
        if (DriverStation.getAlliance().get() == Alliance.Blue) {
            return LimelightHelpers.getBotPose3d_wpiBlue(CAMERA_NAME);
        } else if (DriverStation.getAlliance().get() == Alliance.Red) {
            return LimelightHelpers.getBotPose3d_wpiRed(CAMERA_NAME);
        }
        DriverStation.reportWarning("Invalid Team", false);
        return LimelightHelpers.getBotPose3d_wpiBlue(CAMERA_NAME); //send blue by default
    }



    /*
     *
     * Custom Helpers
     * 
     * 
     */









   /*
     *
     * Utility Wrappers
     * 
     * 
     */

    /**
     * @return The latest LL results as a LimelightResults object.
     */
    private LimelightResults retrieveJSON() {
            return LimelightHelpers.getLatestResults(CAMERA_NAME);
    }

    /**
     * @param pipelineIndex use pipeline indexes in {@link VisionConfig} //TODO: come back
     */
    public void setLimelightPipeline(int pipelineIndex) {
        LimelightHelpers.setPipelineIndex(CAMERA_NAME, pipelineIndex);
    }

    /**
     * Sets the LED mode of the LL.
     * 
     * @param enabled true to enable the LED mode, false to disable it
     */
    public void setLEDMode(boolean enabled) {
        if(enabled) {
            LimelightHelpers.setLEDMode_ForceOn(CAMERA_NAME);
        } else {
            LimelightHelpers.setLEDMode_ForceOff(CAMERA_NAME);
        }
    }

    /**
     * Checks if the camera is connected by looking for the JSON string returned in NetworkTables.
     */
    public boolean isCameraConnected() {
        return !NetworkTableInstance.getDefault()
                        .getTable(CAMERA_NAME)
                        .getEntry("json")
                        .getString("")
                        .equals("");
    }

    /**
     * Prints the vision, estimated, and odometry pose to SmartDashboard
     */
    public void printDebug() {
        Pose3d botPose3d = getAlliancePose();
            SmartDashboard.putString("LimelightX", df.format(botPose3d.getTranslation().getX()));
            SmartDashboard.putString("LimelightY", df.format(botPose3d.getTranslation().getY()));
            SmartDashboard.putString("LimelightZ", df.format(botPose3d.getTranslation().getZ()));
            SmartDashboard.putString(
                    "LimelightRoll",
                    df.format(Units.radiansToDegrees(botPose3d.getRotation().getX())));
            SmartDashboard.putString(
                    "LimelightPitch",
                    df.format(Units.radiansToDegrees(botPose3d.getRotation().getY())));
            SmartDashboard.putString(
                    "LimelightYaw",
                    df.format(Units.radiansToDegrees(botPose3d.getRotation().getZ())));
        // SmartDashboard.putString("EstimatedPoseX", df.format(Robot.pose.getEstimatedPose().getX()));
        // SmartDashboard.putString("EstimatedPoseY", df.format(Robot.pose.getEstimatedPose().getY()));
        // SmartDashboard.putString(
                // "EstimatedPoseTheta", df.format(Robot.pose.getHeading().getDegrees()));
    }


    //TODO: isEstimateReady, toPose2d
}
