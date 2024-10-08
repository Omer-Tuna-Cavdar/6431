package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;

public class ToggleIntakeCommand extends Command {
    private final Intake intake;
    private double targetPosition;
    private boolean isOpening;

    public ToggleIntakeCommand(Intake intake) {
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (intake.isIntakeClosed()) {
            // Intake is closed; open it and start the rollers
            targetPosition = Constants.INTAKE_OPEN_POSITION;
            intake.setPivotPosition(targetPosition);
            intake.runIntake(Constants.INTAKE_ROLLER_SPEED);
            isOpening = true;
        } else if (intake.isIntakeOpen()) {
            // Intake is open; stop the rollers and close it
            targetPosition = Constants.INTAKE_CLOSED_POSITION;
            intake.setPivotPosition(targetPosition);
            intake.stopIntake();
            isOpening = false;
        } else {
            // Intake is in between; decide action based on current position
            double currentPosition = intake.getPivotPosition();
            double midpoint = (Constants.INTAKE_OPEN_POSITION + Constants.INTAKE_CLOSED_POSITION) / 2.0;
            if (currentPosition < midpoint) {
                // Closer to closed position; open it
                targetPosition = Constants.INTAKE_OPEN_POSITION;
                intake.setPivotPosition(targetPosition);
                intake.runIntake(Constants.INTAKE_ROLLER_SPEED);
                isOpening = true;
            } else {
                // Closer to open position; close it
                targetPosition = Constants.INTAKE_CLOSED_POSITION;
                intake.setPivotPosition(targetPosition);
                intake.stopIntake();
                isOpening = false;
            }
        }
    }

    @Override
    public void execute() {
        // No additional actions needed; control is handled in the Intake subsystem's periodic method
    }

    @Override
    public boolean isFinished() {
        // Command finishes when the pivot reaches the target position
        return intake.isPivotAtPosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the pivot motor to prevent further movement
        intake.stopPivot();
        // If the intake was closing, ensure the rollers are stopped
        if (!isOpening) {
            intake.stopIntake();
        }
    }
}