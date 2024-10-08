// Drivetrain.java

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Drivetrain extends SubsystemBase {
    // Motor Controllers
    private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.ktankdriveMotor1lId, MotorType.kBrushless);
    private final CANSparkMax leftRearMotor = new CANSparkMax(Constants.ktankdriveMotor2lId, MotorType.kBrushless);
    private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.ktankdriveMotor1rId, MotorType.kBrushless);
    private final CANSparkMax rightRearMotor = new CANSparkMax(Constants.ktankdriveMotor2rId, MotorType.kBrushless);

    // Differential Drive
    private final DifferentialDrive differentialDrive;

    public Drivetrain() {

        // Motor Configuration
        leftFrontMotor.setInverted(Constants.LEFT_MOTOR_INVERTED);
        rightFrontMotor.setInverted(Constants.RIGHT_MOTOR_INVERTED);
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

        // Initialize DifferentialDrive
        differentialDrive = new DifferentialDrive(leftFrontMotor, rightFrontMotor);

        // Safety Configuration
        differentialDrive.setSafetyEnabled(true);
        differentialDrive.setExpiration(0.1);
        differentialDrive.setMaxOutput(1.0);
    }

    public void arcadeDrive(double fwd, double rot) {
        differentialDrive.arcadeDrive(fwd, rot, true);
    }

    /**
     * Stops all drivetrain motors.
     */
    public void stop() {
        differentialDrive.stopMotor();
    }

    @Override
    public void periodic() {
        // Called periodically during robot operation
    }
}