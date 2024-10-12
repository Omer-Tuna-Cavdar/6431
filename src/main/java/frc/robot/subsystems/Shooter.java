
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends SubsystemBase {
    private final CANSparkMax ShooterL = new CANSparkMax(Constants.kShooterRId, MotorType.kBrushless);
    private final CANSparkMax ShooterR = new CANSparkMax(Constants.kShooterLId, MotorType.kBrushless);
    private final PIDController shooterPidController = new PIDController(Constants.kshooterP, Constants.kshooterI, Constants.kshooterD);
    public Shooter(){
        shooterPidController.setTolerance(Constants.kshooterpositionTolerance, Constants.kshootervelocityTolerance);
        ShooterL.setInverted(Constants.kShooterLInverted);
        ShooterR.setInverted(Constants.kShooterRInverted);
        ShooterL.setIdleMode(IdleMode.kCoast);
        ShooterR.setIdleMode(IdleMode.kCoast);
        ShooterL.setSmartCurrentLimit(40);
        ShooterR.setSmartCurrentLimit(40);
        ShooterL.follow(ShooterR);
    }
       public void runShooter(double RPM) {
        shooterPidController.setSetpoint(RPM);
    }

    public boolean isAtTargetRPM(double targetRPM) {
        double currentRPM = getShooterRPM();
        return Math.abs(currentRPM - targetRPM) <= Constants.kshooterRPMTolerance;
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
