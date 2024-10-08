// Drivetrain.java

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import com.revrobotics.CANSparkMax;

public class Drivetrain extends SubsystemBase {
    // Motor Controllers
    private final CANSparkMax leftFrontMotor = new CANSparkMax(Constants.ktankdriveMotor1lId);
    private final CANSparkMax leftRearMotor = new CANSparkMax(Constants.LEFT_REAR_MOTOR_ID);
    private final CANSparkMax rightFrontMotor = new CANSparkMax(Constants.RIGHT_FRONT_MOTOR_ID);
    private final CANSparkMax rightRearMotor = new CANSparkMax(Constants.RIGHT_REAR_MOTOR_ID);

    // Differential Drive
    private final DifferentialDrive differentialDrive;

    public Drivetrain() {
        // Motor Configuration
        leftFrontMotor.setInverted(Constants.LEFT_MOTOR_INVERTED);
        rightFrontMotor.setInverted(Constants.RIGHT_MOTOR_INVERTED);

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

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd The forward/backward speed [-1.0 to 1.0]
     * @param rot The rotation rate [-1.0 to 1.0]
     */
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