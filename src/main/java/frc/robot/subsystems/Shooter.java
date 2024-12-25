
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    private final CANSparkMax ShooterL = new CANSparkMax(Constants.kShooterLId, MotorType.kBrushless);
    private final CANSparkMax ShooterR = new CANSparkMax(Constants.kShooterRId, MotorType.kBrushless);
    private final PIDController shooterPidController = new PIDController(Constants.kShooterP, Constants.kShooterI, Constants.kShooterD);
    public Shooter(){
        shooterPidController.setTolerance(Constants.kShooterPositionTolerance, Constants.kShooterVelocityTolerance);
        ShooterL.setIdleMode(IdleMode.kCoast);
        ShooterR.setIdleMode(IdleMode.kCoast);
        ShooterL.setSmartCurrentLimit(40);
        ShooterR.setSmartCurrentLimit(40);
        }
       public void runShooter(double RPM) {
        double outputR = shooterPidController.calculate(ShooterR.getEncoder().getPosition(), RPM);
        double outputL = shooterPidController.calculate(ShooterL.getEncoder().getPosition(), RPM);

        // Implement clamping manually
        if (outputR > 0.7) {
            outputR = 0.7;
        } else if (outputR < -0.7) {
            outputR = -0.7;
        }
       if (outputL > 0.7) {
            outputL = 0.7;
        } else if (outputL < -0.7) {
            outputL = -0.7;
        } 
        
        ShooterR.set(outputR);
        ShooterL.set(outputL);
    }

    public boolean isAtTargetRPM(double targetRPM) {
        double currentRPM = getShooterRPM();
        return Math.abs(currentRPM - targetRPM) <= Constants.kShooterVelocityTolerance;
    }

    public void stopShooter() {
        ShooterR.set(0);
        ShooterL.set(0);
    }
    public double getShooterRPM(){
        return ShooterR.getEncoder().getVelocity();
    }
    public double getShooterAMPS(){
        return ShooterR.getOutputCurrent();
    }
    public double getShooterVoltage(){
        return ShooterR.getBusVoltage();
    }
    public void periodic(){
        SmartDashboard.putNumber("Shooter Velocity", getShooterVelocity());
        SmartDashboard.putNumber("Shooter AMPS", getShooterAMPS());
        SmartDashboard.putNumber("Shooter Voltage", getShooterVoltage());

    }
    public double getShooterVelocity(){
        return ShooterR.getEncoder().getVelocity();
    }
}
