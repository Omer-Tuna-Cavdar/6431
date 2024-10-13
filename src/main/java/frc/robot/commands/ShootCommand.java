package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;

public class ShootCommand extends Command {
    private final double targetRPM;
    private boolean projectileReleased;
    private final Timer timer = new Timer();

    public ShootCommand(Shooter shooter, Intake intake, double targetRPM) {;
        this.targetRPM = targetRPM;
        addRequirements(shooter, intake);
        projectileReleased = false;
        timer.reset();
    }

    @Override
    public void initialize() {
        // Start the shooter flywheels to reach the target RPM
        Constants.shooterSubsytem.runShooter(targetRPM);
        timer.reset();
        timer.start();
        if (timer.hasElapsed(1.0)) {
            // If the shooter is ready, run the intake to release the projectile
            Constants.intakeSubsystem.runIntake(Constants.intakerollerrealesespeed);
            projectileReleased = true;
        }
        if (projectileReleased){
            Constants.shooterSubsytem.stopShooter();
            Constants.intakeSubsystem.stopIntake();
        }

    }

    @Override
    public void execute() {
        // Check if the shooter flywheels are at the target velocity
      
    }

    @Override
    public void end(boolean interrupted) {
        // Stop both the shooter and the intake after shooting
        Constants.shooterSubsytem.stopShooter();
        Constants.intakeSubsystem.stopIntake();
        timer.stop();
    }

    @Override
    public boolean isFinished() {
        // Command completes after releasing the projectile
        return projectileReleased && timer.hasElapsed(2);
    }
}