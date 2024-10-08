// In Intake.java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;

public class Intake extends SubsystemBase {
    // Motor Controllers
    private final CANSparkMax RollerMotor = new CANSparkMax(Constants.kintakeRollerId, MotorType.kBrushless);
    private final CANSparkMax PivotMotor = new CANSparkMax(Constants.kintakePivotid, MotorType.kBrushless);
    private final DutyCycleEncoder IntakeBore = new DutyCycleEncoder(Constants.IntakeBoreID);

    private final PIDController pivotPIDController = new PIDController(Constants.pivotkP, Constants.pivotkI, Constants.pivotkD);

    public Intake() {
        RollerMotor.setInverted(Constants.intakerollerinverted);
        PivotMotor.setInverted(Constants.intakepivotinverted);
        RollerMotor.setIdleMode(IdleMode.kCoast);
        PivotMotor.setIdleMode(IdleMode.kBrake);
        PivotMotor.setSmartCurrentLimit(40);
        RollerMotor.setSmartCurrentLimit(40);

        IntakeBore.setDistancePerRotation(360.0); // Configure encoder to output degrees

        pivotPIDController.setTolerance(Constants.positionTolerance);
        pivotPIDController.setSetpoint(getPivotPosition()); // Initialize setpoint to current position
    }

    // Intake control methods
    public void runIntake(double speed) {
        RollerMotor.set(speed);
    }

    public void stopIntake() {
        RollerMotor.set(0);
    }

    // Pivot control methods
    public void setPivotPosition(double position) {
        pivotPIDController.setSetpoint(position);
    }

    public void stopPivot() {
        PivotMotor.set(0);
    }

    public double getPivotPosition() {
        return IntakeBore.getDistance(); // Returns position in degrees
    }

    public boolean isPivotAtPosition(double targetPosition) {
        return Math.abs(getPivotPosition() - targetPosition) <= Constants.positionTolerance;
    }

    public boolean isIntakeOpen() {
        return isPivotAtPosition(Constants.INTAKE_OPEN_POSITION);
    }

    public boolean isIntakeClosed() {
        return isPivotAtPosition(Constants.INTAKE_CLOSED_POSITION);
    }

    @Override
    public void periodic() {
        double currentPosition = getPivotPosition();
        double output = pivotPIDController.calculate(currentPosition);
        output = MathUtil.clamp(output, -0.5, 0.5);
        PivotMotor.set(output);
    }

    public void resetPivotEncoder() {
        IntakeBore.reset();
        pivotPIDController.reset();
    }
}