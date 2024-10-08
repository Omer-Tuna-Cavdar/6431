// Drivetrain.java

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
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
    
    // Differential Drive
    private final DifferentialDrive differentialDrive;
    private DifferentialDriveOdometry odometry;
    private DifferentialDriveKinematics kinematics;

    public Drivetrain() {
        rightEncoder.setPositionConversionFactor(Constants.conversionFactor);
        leftEncoder.setPositionConversionFactor(Constants.conversionFactor);
        rightEncoder.setVelocityConversionFactor(Constants.conversionFactor/60);
        leftEncoder.setVelocityConversionFactor(Constants.conversionFactor/60);
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
        leftFrontMotor.burnFlash();
        leftRearMotor.burnFlash();

        // Initialize DifferentialDrive
        differentialDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

        // Safety Configuration
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(1.0);
        // Kinematics Odometry and Gyro
        kinematics = new DifferentialDriveKinematics(Constants.trackwWidth);
        odometry = new DifferentialDriveOdometry(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        odometry.resetPosition(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), new Pose2d());
        zeroGyro();
        resetDriveEncoders();
        RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );

    }

    public void arcadeDrive(double fwd, double rot) {
        differentialDrive.arcadeDrive(fwd, rot, true);
    }
    public void driveRobotRelative(ChassisSpeeds ChassisSpeeds){
        DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(ChassisSpeeds);
    
    double leftSpeed = wheelSpeeds.leftMetersPerSecond;
    double rightSpeed = wheelSpeeds.rightMetersPerSecond;
    
    // Use the arcadeDrive method of differentialDrive to drive using these wheel speeds
    differentialDrive.tankDrive(leftSpeed, rightSpeed);

    } 
    public void resetDriveEncoders(){
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }
    public void stop() {
        differentialDrive.stopMotor();
    }
    public double getRightEncoderPosition(){
        return rightEncoder.getPosition();
    }
    public double getLeftEncoderPosition(){
        return leftEncoder.getPosition();
    }
      public double getRightEncoderVelocity(){
        return rightEncoder.getVelocity();
    }
    public double getLeftEncoderVelocity(){
        return leftEncoder.getVelocity();
    }
    public double getHeading(){
        return gyro.getRotation2d().getDegrees();
    }
    public double getTurnRate(){
        return -gyro.getRate();//Turn Rate in Degrees per sec
    }
    public void zeroGyro(){
        gyro.reset();
    }
    public Pose2d getPose(){
        return odometry.getPoseMeters();
    }
    public void resetPose(Pose2d pose2d){
        resetDriveEncoders();
        odometry.resetPosition(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition(), pose2d);
    }
    public DifferentialDriveWheelSpeeds getWheelspSpeeds(){
        return new DifferentialDriveWheelSpeeds(getLeftEncoderPosition(), getRightEncoderVelocity());
    }
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return kinematics.toChassisSpeeds(getWheelspSpeeds());
    }
    public void tankDriveVolts(double leftVolts, double rightVolts){
        leftFrontMotor.setVoltage(leftVolts);
        leftRearMotor.setVoltage(leftVolts);
        rightFrontMotor.setVoltage(leftVolts);
        rightRearMotor.setVoltage(leftVolts);
        differentialDrive.feed();
    }
    public double getAvarageEncoderDistance(){
        return ((getLeftEncoderPosition()+getRightEncoderPosition())/2.0);
    }
    public RelativeEncoder getLeftEncoder(){
        return leftEncoder;
    }
    public RelativeEncoder getRRigEncoder(){
        return rightEncoder;
    }
    public void setMaxOutput(double maxOutput){
        differentialDrive.setMaxOutput(maxOutput);
    }
    public Pigeon2 getGyro(){
        return gyro;
    }

    public void periodic() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());
        SmartDashboard.putNumber("Drive Left Encoder Pos", getLeftEncoderPosition());
        SmartDashboard.putNumber("Drive Right Encoder Pos", getRightEncoderPosition());
        SmartDashboard.putNumber("Gyro Heading", getHeading());
    }
}