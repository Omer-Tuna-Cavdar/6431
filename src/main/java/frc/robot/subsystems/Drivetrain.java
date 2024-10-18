// Drivetrain.java

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.sim.Pigeon2SimState;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// Import simulation classes
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Imports for Data Logging and NetworkTables
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

// Import REVPhysicsSim for SparkMax simulation
import com.revrobotics.REVPhysicsSim;

public class Drivetrain extends SubsystemBase {
    // Motor controllers
    private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.kTankDriveMotor1lId, MotorType.kBrushless);
    private final CANSparkMax leftRearMotor = new CANSparkMax(Constants.kTankDriveMotor2lId, MotorType.kBrushless);
    private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.kTankDriveMotor1rId, MotorType.kBrushless);
    private final CANSparkMax rightRearMotor = new CANSparkMax(Constants.kTankDriveMotor2rId, MotorType.kBrushless);

    // Encoders
    private final RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
    private final RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();

    // Gyro
    private final Pigeon2 gyro = new Pigeon2(Constants.GyroID);
    private Pigeon2SimState gyroSim; // For simulation

    // Kinematics and Odometry
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.TRACK_WIDTH_METERS);
    private final DifferentialDriveOdometry odometry;

    // Differential Drive
    private final DifferentialDrive differentialDrive;

    // Pose variables
    private Pose3d currentPose3d;
    private Pose2d currentPose2d;

    // NetworkTables publishers
    private final StructPublisher<Pose3d> posePublisher;
    private final StructPublisher<Pose2d> pose2dPublisher;

    private final DoublePublisher pose2dXPublisher;
    private final DoublePublisher pose2dYPublisher;
    private final DoublePublisher pose2dRotationPublisher;

    // DataLog entries
    private final DoubleLogEntry leftEncoderPositionLog;
    private final DoubleLogEntry rightEncoderPositionLog;
    private final DoubleLogEntry leftEncoderVelocityLog;
    private final DoubleLogEntry rightEncoderVelocityLog;
    private final DoubleLogEntry gyroAngleLog;

    private final DoubleLogEntry pose2dXLog;
    private final DoubleLogEntry pose2dYLog;
    private final DoubleLogEntry pose2dRotationLog;

    private final DoubleLogEntry pose3dXLog;
    private final DoubleLogEntry pose3dYLog;
    private final DoubleLogEntry pose3dZLog;
    private final DoubleLogEntry pose3dRollLog;
    private final DoubleLogEntry pose3dPitchLog;
    private final DoubleLogEntry pose3dYawLog;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
                Constants.kS, Constants.kV, Constants.kA);
    private final PIDController leftPIDController = new PIDController(
    Constants.LeftDrivekP, Constants.LeftDrivekI, Constants.LeftDrivekD);
    private final PIDController rightPIDController = new PIDController(
                Constants.RightDrivekP, Constants.RightDrivekI, Constants.RightDrivekD);

    // Simulation components
    private DifferentialDrivetrainSim drivetrainSimulator;
    private Field2d fieldSim;

    public Drivetrain() {
                    
               AutoBuilder.configureLTV(
                this::getPose,
                this::resetOdometry,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                0.1, // Increase time step for smoother updates in auto
                new ReplanningConfig(),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    
        // Motor configuration
        leftFrontMotor.restoreFactoryDefaults();
        leftRearMotor.restoreFactoryDefaults();
        rightFrontMotor.restoreFactoryDefaults();
        rightRearMotor.restoreFactoryDefaults();

        leftFrontMotor.setInverted(Constants.LEFT_DRIVE_MOTOR_INVERTED);
        rightFrontMotor.setInverted(Constants.RIGHT_DRIVE_MOTOR_INVERTED);

        leftRearMotor.follow(leftFrontMotor);
        rightRearMotor.follow(rightFrontMotor);
        leftFrontMotor.getPIDController().setP(Constants.LeftDrivekP);
        rightFrontMotor.getPIDController().setP(Constants.RightDrivekP);
        leftRearMotor.getPIDController().setP(Constants.LeftDrivekP);
        rightRearMotor.getPIDController().setP(Constants.RightDrivekP);


        // Set idle modes
        leftFrontMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        leftRearMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightFrontMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        rightRearMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);

        // Current limits
        leftFrontMotor.setSmartCurrentLimit(40);
        leftRearMotor.setSmartCurrentLimit(40);
        rightFrontMotor.setSmartCurrentLimit(40);
        rightRearMotor.setSmartCurrentLimit(40);

        // Encoder configuration
        leftEncoder.setPositionConversionFactor(Constants.ENCODER_POSITION_CONVERSION_FACTOR);
        leftEncoder.setVelocityConversionFactor(Constants.ENCODER_VELOCITY_CONVERSION_FACTOR);
        rightEncoder.setPositionConversionFactor(Constants.ENCODER_POSITION_CONVERSION_FACTOR);
        rightEncoder.setVelocityConversionFactor(Constants.ENCODER_VELOCITY_CONVERSION_FACTOR);

        // DifferentialDrive setup
        differentialDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);
        differentialDrive.setSafetyEnabled(false);

        // Odometry setup
        odometry = new DifferentialDriveOdometry(getHeading(), leftEncoder.getPosition(), rightEncoder.getPosition());

        // Zero sensors
        resetEncoders();
        zeroGyro();

        // Initialize pose variables
        currentPose3d = new Pose3d();
        currentPose2d = new Pose2d();

        // Initialize NetworkTables publishers
        posePublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Drivetrain/Pose3d", Pose3d.struct).publish();
        pose2dPublisher = NetworkTableInstance.getDefault()
            .getStructTopic("Drivetrain/Pose2d", Pose2d.struct).publish();

        pose2dXPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("Drivetrain/Pose2d/X").publish();
        pose2dYPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("Drivetrain/Pose2d/Y").publish();
        pose2dRotationPublisher = NetworkTableInstance.getDefault()
            .getDoubleTopic("Drivetrain/Pose2d/Rotation").publish();

        // Initialize DataLog entries
        DataLogManager.start();
        DataLogManager.logNetworkTables(false); // Optional: log NetworkTables data

        leftEncoderPositionLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/leftEncoderPosition");
        rightEncoderPositionLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/rightEncoderPosition");
        leftEncoderVelocityLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/leftEncoderVelocity");
        rightEncoderVelocityLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/rightEncoderVelocity");
        gyroAngleLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/gyroAngle");

        pose2dXLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/pose2d/x");
        pose2dYLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/pose2d/y");
        pose2dRotationLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/pose2d/rotation");

        pose3dXLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/pose3d/x");
        pose3dYLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/pose3d/y");
        pose3dZLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/pose3d/z");
        pose3dRollLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/pose3d/roll");
        pose3dPitchLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/pose3d/pitch");
        pose3dYawLog = new DoubleLogEntry(DataLogManager.getLog(), "/drivetrain/pose3d/yaw");

        // Simulation support
        if (RobotBase.isSimulation()) {
            gyroSim = gyro.getSimState();

            // Create the drivetrain simulator
            drivetrainSimulator = new DifferentialDrivetrainSim(
                LinearSystemId.identifyDrivetrainSystem(
                    Constants.kvVoltSecondsPerMeter,
                    Constants.kaVoltSecondsSquaredPerMeter,
                    Constants.kvVoltSecondsPerRadian,
                    Constants.kaVoltSecondsSquaredPerRadian),
                DCMotor.getNEO(2), // 2 NEO motors per side
                Constants.GEAR_RATIO,
                Constants.TRACK_WIDTH_METERS,
                Constants.WHEEL_DIAMETER_METERS / 2,
                null);

            // Create field simulation
            fieldSim = new Field2d();
            SmartDashboard.putData("Field", fieldSim);

            // Add the SparkMax controllers to the REVPhysicsSim
            REVPhysicsSim.getInstance().addSparkMax(leftFrontMotor, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(rightFrontMotor, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(leftRearMotor, DCMotor.getNEO(1));
            REVPhysicsSim.getInstance().addSparkMax(rightRearMotor, DCMotor.getNEO(1));
        }
    }

    public void arcadeDrive(double fwd, double rot) {
        differentialDrive.arcadeDrive(fwd, rot);
    }

    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftFrontMotor.setVoltage(leftVolts);
        rightFrontMotor.setVoltage(rightVolts);
        differentialDrive.feed();
    }

    public void resetEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(gyro.getYaw().getValue());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public Pose3d getPose3d() {
        Pose2d pose2d = getPose();
        double x = pose2d.getX();
        double y = pose2d.getY();
        double z = 0.0; // Assuming flat field

        // Get roll, pitch, and yaw in degrees
        double rollDegrees = gyro.getRoll().getValue();
        double pitchDegrees = gyro.getPitch().getValue();
        double yawDegrees = gyro.getYaw().getValue();

        // Convert degrees to radians for Rotation3d
        Rotation3d rotation3d = new Rotation3d(
                Math.toRadians(rollDegrees),
                Math.toRadians(pitchDegrees),
                Math.toRadians(yawDegrees)
        );

        return new Pose3d(x, y, z, rotation3d);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getVelocity(), rightEncoder.getVelocity());
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public void setOutput(double leftVolts, double rightVolts) {
        tankDriveVolts(leftVolts, rightVolts);
    }

    @Override
    public void periodic() {
        // Update odometry

        // Get current poses
        currentPose2d = getPose();
        currentPose3d = getPose3d();

        // Publish to NetworkTables
        posePublisher.set(currentPose3d);
        pose2dPublisher.set(currentPose2d);

        pose2dXPublisher.set(currentPose2d.getX());
        pose2dYPublisher.set(currentPose2d.getY());
        pose2dRotationPublisher.set(currentPose2d.getRotation().getDegrees());

        // Log encoder data
        leftEncoderPositionLog.append(leftEncoder.getPosition());
        rightEncoderPositionLog.append(rightEncoder.getPosition());
        leftEncoderVelocityLog.append(leftEncoder.getVelocity());
        rightEncoderVelocityLog.append(rightEncoder.getVelocity());

        // Log gyro angle
        gyroAngleLog.append(getHeading().getDegrees());

        // Log Pose2d components
        pose2dXLog.append(currentPose2d.getX());
        pose2dYLog.append(currentPose2d.getY());
        pose2dRotationLog.append(currentPose2d.getRotation().getDegrees());

        // Log Pose3d components
        pose3dXLog.append(currentPose3d.getX());
        pose3dYLog.append(currentPose3d.getY());
        pose3dZLog.append(currentPose3d.getZ());
        pose3dRollLog.append(gyro.getRoll().getValue());
        pose3dPitchLog.append(gyro.getPitch().getValue());
        pose3dYawLog.append(gyro.getYaw().getValue());

        // Update field simulation
        if (fieldSim != null) {
            fieldSim.setRobotPose(currentPose2d);
        }
    }

    @Override
    public void simulationPeriodic() {
        // Run the REVPhysicsSim for SparkMax devices
        REVPhysicsSim.getInstance().run();

        // Set inputs to the drivetrain simulator
        drivetrainSimulator.setInputs(
            leftFrontMotor.get() * RobotController.getBatteryVoltage(),
            rightFrontMotor.get() * RobotController.getBatteryVoltage());

        // Advance the simulation by 20 ms
        drivetrainSimulator.update(0.02);

        // Update the gyro simulation
        gyroSim.setRawYaw(drivetrainSimulator.getHeading().getDegrees());
    }

    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(getHeading(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose);
    }

    public void stop() {
        leftFrontMotor.stopMotor();
        leftRearMotor.stopMotor();
        rightFrontMotor.stopMotor();
        rightRearMotor.stopMotor();
        differentialDrive.feed();
    }
    public DifferentialDriveOdometry getOdometry(){
        return odometry;
    }
    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition();
    }
    
    public double getRightEncoderPosition() {
        return rightEncoder.getPosition();
    }
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }
    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        // Step 1: Convert ChassisSpeeds to DifferentialDriveWheelSpeeds
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
    
        // Step 2: Calculate Feedforward voltages
        double leftFeedforward = feedforward.calculate(wheelSpeeds.leftMetersPerSecond);
        double rightFeedforward = feedforward.calculate(wheelSpeeds.rightMetersPerSecond);
    
        // Step 3: Calculate PID controller outputs
        double leftOutput = leftPIDController.calculate(leftEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond);
        double rightOutput = rightPIDController.calculate(rightEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond);
    
        // Step 4: Combine Feedforward and PID outputs
        double leftVolts = leftFeedforward + leftOutput;
        double rightVolts = rightFeedforward + rightOutput;
    
        // Step 5: Set motor voltages
        tankDriveVolts(leftVolts, rightVolts);
    }
    
}
