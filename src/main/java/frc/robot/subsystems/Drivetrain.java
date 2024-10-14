// Drivetrain.java

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Drivetrain extends SubsystemBase {
    // Motor Controllers
    private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.ktankdriveMotor1lId, MotorType.kBrushless);
    private final CANSparkMax leftRearMotor = new CANSparkMax(Constants.ktankdriveMotor2lId, MotorType.kBrushless);
    private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.ktankdriveMotor1rId, MotorType.kBrushless);
    private final CANSparkMax rightRearMotor = new CANSparkMax(Constants.ktankdriveMotor2rId, MotorType.kBrushless);
    private final RelativeEncoder rightEncoder = rightFrontMotor.getEncoder();
    private final RelativeEncoder leftEncoder = leftFrontMotor.getEncoder();
    private final Pigeon2 gyro = new Pigeon2(Constants.GyroID);

    // PID Controllers
    private final SparkPIDController leftPIDController = leftFrontMotor.getPIDController();
    private final SparkPIDController rightPIDController = rightFrontMotor.getPIDController();

    // Differential Drive
    private final DifferentialDrive differentialDrive;
    private DifferentialDriveOdometry odometry;
    private DifferentialDriveKinematics kinematics;
    ReplanningConfig config = new ReplanningConfig();

    public Drivetrain() {
        rightEncoder.setPositionConversionFactor(Constants.conversionFactor);
        leftEncoder.setPositionConversionFactor(Constants.conversionFactor);
        rightEncoder.setVelocityConversionFactor(Constants.conversionFactor / 60);
        leftEncoder.setVelocityConversionFactor(Constants.conversionFactor / 60);

        // Motor Configuration
        leftFrontMotor.setInverted(Constants.LEFT_DriveMOTOR_INVERTED);
        rightFrontMotor.setInverted(Constants.RIGHT_DriveMOTOR_INVERTED);
        leftFrontMotor.setIdleMode(IdleMode.kBrake);
        leftRearMotor.setIdleMode(IdleMode.kBrake);
        rightFrontMotor.setIdleMode(IdleMode.kBrake);
        rightRearMotor.setIdleMode(IdleMode.kBrake);
        leftFrontMotor.setSmartCurrentLimit(40);
        leftRearMotor.setSmartCurrentLimit(40);
        rightFrontMotor.setSmartCurrentLimit(40);
        rightRearMotor.setSmartCurrentLimit(40);
        // Rear motors follow front motors
        leftRearMotor.follow(leftFrontMotor);
        rightRearMotor.follow(rightFrontMotor);
        leftFrontMotor.burnFlash();
        rightFrontMotor.burnFlash();
        leftRearMotor.burnFlash();
        rightRearMotor.burnFlash();

        // Initialize DifferentialDrive
        differentialDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

        // Safety Configuration
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.02);
        differentialDrive.setMaxOutput(0.8);

        // Kinematics, Odometry, and Gyro
        kinematics = new DifferentialDriveKinematics(Constants.trackwWidth);
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        odometry.resetPosition(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), new Pose2d());
        zeroGyro();
        resetDriveEncoders();

        // Configure PID Controllers
        leftPIDController.setP(Constants.LeftDrivekP);
        leftPIDController.setI(Constants.LeftDrivekI);
        leftPIDController.setD(Constants.LeftDrivekD);
        leftPIDController.setFF(Constants.LeftDrivekFF);

        rightPIDController.setP(Constants.RightDrivekP);
        rightPIDController.setI(Constants.RightDrivekI);
        rightPIDController.setD(Constants.RightDrivekD);
        rightPIDController.setFF(Constants.RightDrivekFF);

        // Configure AutoBuilder last
        AutoBuilder.configureLTV(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                this::driveRobotRelative,
                0.02,
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
    }

    public void arcadeDrive(double fwd, double rot) {
        differentialDrive.arcadeDrive(fwd, rot, true);
    }

    public void driveRobotRelative(ChassisSpeeds chassisSpeeds) {
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(chassisSpeeds);
        double leftSpeed = wheelSpeeds.leftMetersPerSecond;
        double rightSpeed = wheelSpeeds.rightMetersPerSecond;

        // Use PID to set motor velocities
        leftPIDController.setReference(leftSpeed, ControlType.kVelocity);
        rightPIDController.setReference(rightSpeed, ControlType.kVelocity);
    }

    public void resetDriveEncoders() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public void stop() {
        differentialDrive.stopMotor();
    }

    public double getRightEncoderPosition() {
        return rightEncoder.getPosition();
    }

    public double getLeftEncoderPosition() {
        return leftEncoder.getPosition();
    }

    public double getRightEncoderVelocity() {
        return rightEncoder.getVelocity();
    }

    public double getLeftEncoderVelocity() {
        return leftEncoder.getVelocity();
    }

    public double getHeading() {
        return gyro.getRotation2d().getDegrees();
    }

    public double getTurnRate() {
        return -gyro.getRate(); // Turn Rate in Degrees per sec
    }

    public void zeroGyro() {
        gyro.reset();
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose2d) {
        resetDriveEncoders();
        odometry.resetPosition(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose2d);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }

    public double getDriveTrainAMPS() {
        return leftFrontMotor.getOutputCurrent() + leftRearMotor.getOutputCurrent() + rightFrontMotor.getOutputCurrent() + rightRearMotor.getOutputCurrent();
    }

    public double getLeftDriveTrainAMPS() {
        return leftFrontMotor.getOutputCurrent() + leftRearMotor.getOutputCurrent();
    }

    public double getRightDriveTrainAMPS() {
        return rightFrontMotor.getOutputCurrent() + rightRearMotor.getOutputCurrent();
    }

    public double getDriveTrainVoltage() {
        return leftFrontMotor.getBusVoltage() + leftRearMotor.getBusVoltage() + rightFrontMotor.getBusVoltage() + rightRearMotor.getBusVoltage();
    }

    public double getLeftDriveTrainVoltage() {
        return leftFrontMotor.getBusVoltage() + leftRearMotor.getBusVoltage();
    }

    public double getRightDriveTrainVoltage() {
        return rightFrontMotor.getBusVoltage() + rightRearMotor.getBusVoltage();
    }

    public void periodic() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        SmartDashboard.putNumber("Drive Left Encoder Pos", getLeftEncoderPosition());
        SmartDashboard.putNumber("Drive Right Encoder Pos", getRightEncoderPosition());
        SmartDashboard.putNumber("Gyro Heading", getHeading());
        SmartDashboard.putNumber("Gyro Turn Rate", getTurnRate());
        SmartDashboard.putNumber("Drive Train AMPS", getDriveTrainAMPS());
        SmartDashboard.putNumber("Drive Train Left AMPS", getLeftDriveTrainAMPS());
        SmartDashboard.putNumber("Drive Train Right AMPS", getRightDriveTrainAMPS());
        SmartDashboard.putNumber("Drive Train Right Voltage", getRightDriveTrainVoltage());
        SmartDashboard.putNumber("Drive Train Left Voltage", getLeftDriveTrainVoltage());
        SmartDashboard.putNumber("Drive Train Voltage", getDriveTrainVoltage());
    }
}