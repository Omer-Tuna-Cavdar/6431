package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.MathUtil;
public class Intake extends SubsystemBase {
    // Motor Controllers
    private double targetPivotPosition = Constants.INTAKE_CLOSED_POSITION;
    private final CANSparkMax rollerMotor = new CANSparkMax(Constants.kIntakeRollerId, MotorType.kBrushless);
    private final CANSparkMax PivotMotor = new CANSparkMax(Constants.kIntakePivotId, MotorType.kBrushless);
    private final DutyCycleEncoder IntakeBore = new DutyCycleEncoder(Constants.IntakeBoreID);

    private final PIDController pivotPIDController = new PIDController(Constants.PIVOT_kP, Constants.PIVOT_kI, Constants.PIVOT_kD);

    public Intake() {
        rollerMotor.setInverted(Constants.INTAKE_ROLLER_INVERTED);
        PivotMotor.setInverted(Constants.INTAKE_PIVOT_INVERTED);
        rollerMotor.setIdleMode(IdleMode.kCoast);
        PivotMotor.setIdleMode(IdleMode.kBrake);
        PivotMotor.setSmartCurrentLimit(40);
        rollerMotor.setSmartCurrentLimit(40);
        IntakeBore.setDistancePerRotation(360.0); // Configure encoder to output degrees


        pivotPIDController.setTolerance(1);
        pivotPIDController.setSetpoint(getPivotPosition()); // Initialize setpoint to current position
    }

    // Intake control methods
    public void runIntake(double speed) {
        rollerMotor.set(speed);
    }

    public void stopIntake() {
        rollerMotor.set(0);
    }

    // Pivot control methods
    public void setPivotPosition(double position) {
        double output = pivotPIDController.calculate(IntakeBore.getDistance(), position);
        
        // Implement clamping manually
        if (output > 0.1) {
            output = 0.1;
        } else if (output < -0.1) {
            output = -0.1;
        }
        
        PivotMotor.set(output);
    }
    

    public void stopPivot() {
        PivotMotor.set(0);
    }

    public double getPivotPosition() {
        return IntakeBore.getDistance(); // Returns position in degrees
    }

    public boolean isPivotAtPosition(double targetPosition) {
        return Math.abs(getPivotPosition() - targetPosition) <= Constants.POSITION_TOLERANCE;
    }

    public boolean isIntakeOpen() {
        return isPivotAtPosition(Constants.INTAKE_OPEN_POSITION);
    }

    public boolean isIntakeClosed() {
        return isPivotAtPosition(Constants.INTAKE_CLOSED_POSITION);
    }
    public double getIntskeRollerAMPS() {
        return rollerMotor.getOutputCurrent();
    }   
    public double getIntakePivotAMPS() {
        return PivotMotor.getOutputCurrent();
    }
    public double getrollervoltage() {
        return rollerMotor.getBusVoltage();
    }
    public double getpivotvoltage() {
        return PivotMotor.getBusVoltage();
    }

    public void periodic() {
        double output = pivotPIDController.calculate(getPivotPosition(), targetPivotPosition);

        // Implement clamping
        output = MathUtil.clamp(output, -0.2, 0.2); // Adjust the limits as needed
        if(IntakeBore.getDistance() <= 100 && output < 0){
            output = MathUtil.clamp(output, -0.05, 0.05);
         }
         if(IntakeBore.getDistance() >= 100 && output > 0){
            output = MathUtil.clamp(output, -0.05, 0.05);
         }
        PivotMotor.set(output);

        SmartDashboard.putNumber("intake pivot voltsge", getpivotvoltage());
        SmartDashboard.putNumber("intake roller voltage", getrollervoltage());
        SmartDashboard.putNumber("intake pivot current", getIntakePivotAMPS());
        SmartDashboard.putNumber("intake roller current", getIntskeRollerAMPS());
        SmartDashboard.putNumber("intake encoder", getPivotPosition());
    }

    public void resetPivotEncoder() {
        IntakeBore.reset();
        pivotPIDController.reset();
    }
    public void setPivotTargetPosition(double targetPosition){
        targetPivotPosition = targetPosition;
    }
    public double getPivotTargetPosition(){
        return targetPivotPosition;
    }

}