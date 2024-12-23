package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e., public static). Do not put anything functional in this class.
 */
public final class Constants {

    // Controller Ports
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    // Drivetrain Motor CAN IDs
    public static final int kTankDriveMotor1rId = 9;
    public static final int kTankDriveMotor2rId = 10;
    public static final int kTankDriveMotor1lId = 4;
    public static final int kTankDriveMotor2lId = 41;

    // Gyro CAN ID
    public static final int GyroID = 21;

    // Intake Motor CAN IDs
    public static final int kIntakeRollerId = 5;
    public static final int kIntakePivotId = 33;

    // Intake Sensors
    public static final int IntakeBoreID = 1;

    // Shooter Motor CAN IDs
    public static final int kShooterRId = 42;
    public static final int kShooterLId = 34;

    // Motor Inversions
    public static final boolean LEFT_DRIVE_MOTOR_INVERTED = true;
    public static final boolean RIGHT_DRIVE_MOTOR_INVERTED = false;
    public static final boolean INTAKE_ROLLER_INVERTED = false;
    public static final boolean INTAKE_PIVOT_INVERTED = false;
    public static final boolean kShooterLInverted = false;
    public static final boolean kShooterRInverted = true;

    // Deadband
    public static final double DEADBAND = 0.02;

    // Intake Constants
    public static final double POSITION_TOLERANCE = 3.0;
    public static final double PIVOT_kP = 0.02;
    public static final double PIVOT_kI = 0.0;
    public static final double PIVOT_kD = 0.0;
    public static final double INTAKE_OPEN_POSITION = 213.0;
    public static final double INTAKE_CLOSED_POSITION = 5.0;
    public static final double INTAKE_ROLLER_SPEED = 0.7;
    public static final double INTAKE_ROLLER_RELEASE_SPEED = -0.7;

    // Shooter Constants
    public static final double kShooterP = 0.2;
    public static final double kShooterI = 0.0;
    public static final double kShooterD = 0.0;
    public static final double kShooterPositionTolerance = 3.0;
    public static final double kShooterVelocityTolerance = 1.0;
    public static final double SHOOTER_TARGET_RPM = 6000.0;

    // Drivetrain Physical Constants
    public static final double GEAR_RATIO = 8.46; // Confirm this value based on your robot
    public static final double WHEEL_DIAMETER_METERS = 0.1524; // 6 inches in meters
    public static final double WHEEL_RADIUS_METERS = WHEEL_DIAMETER_METERS / 2.0;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Math.PI * WHEEL_DIAMETER_METERS;
    public static final double ENCODER_POSITION_CONVERSION_FACTOR = WHEEL_CIRCUMFERENCE_METERS / GEAR_RATIO;
    public static final double ENCODER_VELOCITY_CONVERSION_FACTOR = ENCODER_POSITION_CONVERSION_FACTOR / 60.0; // RPM to m/s
    public static final double TRACK_WIDTH_METERS = 0.53133; // Measure and update this value

    // Drivetrain Performance Constants
    public static final double kMaxSpeedMetersPerSecond = 0.10; // Adjust based on your robot's capability
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.01; // Adjust as needed

    // Feedforward constants (Replace these with values from robot characterization)
    public static final double ksVolts = 0.65; // Static gain (example value)
    public static final double kvVoltSecondsPerMeter = 2.75; // Velocity gain (example value)
    public static final double kaVoltSecondsSquaredPerMeter = 0.3; // Acceleration gain (example value)

    // Convert kV and kA to radians
    public static final double kvVoltSecondsPerRadian = kvVoltSecondsPerMeter / WHEEL_RADIUS_METERS;
    public static final double kaVoltSecondsSquaredPerRadian = kaVoltSecondsSquaredPerMeter / WHEEL_RADIUS_METERS;

    // Drive Velocity PID Controller Gains (Replace kPDriveVel with characterization result)
    public static final double kPDriveVel = 1.0; // Proportional gain (example value)
    public static final double kIDriveVel = 0.0; // Integral gain
    public static final double kDDriveVel = 0.0; // Derivative gain

    // Drivetrain Motor PID Constants (These can be adjusted based on testing)
    public static final double LeftDrivekP = 0.1;
    public static final double LeftDrivekI = 0.0;
    public static final double LeftDrivekD = 0.0;
    public static final double LeftDrivekFF = 0.0;

    public static final double RightDrivekP = 0.1;
    public static final double RightDrivekI = 0.0;
    public static final double RightDrivekD = 0.0;
    public static final double RightDrivekFF = 0.0;

    // Subsystems
    public static final Drivetrain drivetrain = new Drivetrain();
    public static final Intake intakeSubsystem = new Intake();
    public static final Shooter shooterSubsystem = new Shooter();
    public static final double RAMSETE_B = 2.0;
    public static final double RAMSETE_ZETA = 0.7;
    public static final double kS = 0.2; // Static friction voltage
    public static final double kV = 0.2; // Velocity proportional voltage
    public static final double kA = 0.2; // Acceleration proportional voltage
}
