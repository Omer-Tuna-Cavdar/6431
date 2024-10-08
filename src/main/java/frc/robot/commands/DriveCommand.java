// DriveCommand.java

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {
    private final Drivetrain drivetrain;
    private final DoubleSupplier forwardSupplier;
    private final DoubleSupplier rotationSupplier;

    public DriveCommand(Drivetrain drivetrain, DoubleSupplier forwardSupplier, DoubleSupplier rotationSupplier) {
        this.drivetrain = drivetrain;
        this.forwardSupplier = forwardSupplier;
        this.rotationSupplier = rotationSupplier;
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double fwd = applyDeadband(forwardSupplier.getAsDouble());
        double rot = applyDeadband(rotationSupplier.getAsDouble());
        drivetrain.arcadeDrive(fwd, rot);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // This command runs until interrupted
    }

    /**
     * Applies a deadband to joystick inputs to prevent drift.
     *
     * @param value The input value from joystick
     * @return The value after applying deadband
     */
    private double applyDeadband(double value) {
        if (Math.abs(value) < Constants.DEADBAND) {
            return 0.0;
        }
        return value;
    }
}