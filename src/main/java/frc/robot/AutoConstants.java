package frc.robot;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 * The AutoConstants class holds constants specific to autonomous operation.
 */
public final class AutoConstants {

    // PID Controller Gains for Trajectory Tracking
    // Note: For a differential drive (tank drive), kPXController and kPYController are typically not used.
    public static final double kPXController = 0.1; // Not used in RamseteCommand for tank drive
    public static final double kPYController = 0.1; // Not used in RamseteCommand for tank drive

    // Heading Controller Gain
    public static final double kPThetaController = 3.0; // Proportional gain for heading controller (tune as needed)

    // Constraints for the Heading Controller (Radians per Second and Radians per Second Squared)
    public static final Constraints kThetaControllerConstraints = new Constraints(
        Math.PI,     // Max angular speed (radians per second)
        Math.PI / 2  // Max angular acceleration (radians per second squared)
    );
}
