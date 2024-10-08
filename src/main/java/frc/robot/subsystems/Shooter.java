
package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;

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
        double currentRPM = ShooterR.getEncoder().getVelocity();
        return Math.abs(currentRPM - targetRPM) <= Constants.kshootervelocityTolerance;
    }

    public void stopShooter() {
        ShooterR.set(0);
        ShooterL.set(0);
    }
}
