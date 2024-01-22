// package frc.robot.auton.config;

// import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
// import com.pathplanner.lib.util.PIDConstants;
// import com.pathplanner.lib.util.ReplanningConfig;

// public class AutonConfig {
// Rotation
//     public static final double kP = 5.0;
//     public static final double kI = 0.0;
//     public static final double kD = 0.0;
//     public static final double maxModuleSpeed = 4.5;
//     public static final double driveBaseRadius = 0.4;

//     public static HolonomicPathFollowerConfig AutonPathFollowerConfig =
//             new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely
//                     // live in your Constants class
//                     new PIDConstants(kP, kI, kD), // Translation PID constants
//                     new PIDConstants(kP, kI, kD), // Rotation PID constants
//                     maxModuleSpeed, // Max module speed, in m/s
//                     driveBaseRadius, // Drive base radius in meters. Distance from robot center
// to
//                     // furthest module.
//                     new ReplanningConfig() // Default path replanning config. See the API for
//                     // the options here
//                     );
// }
