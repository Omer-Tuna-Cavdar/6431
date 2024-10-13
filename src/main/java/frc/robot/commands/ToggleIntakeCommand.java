package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;

public class ToggleIntakeCommand extends Command {;
    private double targetPosition;
    private boolean isOpening;

    public ToggleIntakeCommand(Intake intake) {
      
        addRequirements(intake);
        
    }

    @Override
    public void initialize() {
        if (Constants.intakeSubsystem.isIntakeClosed()) {
            // Intake is closed; open it and start the rollers
            targetPosition = Constants.INTAKE_OPEN_POSITION;
            Constants.intakeSubsystem.setPivotPosition(targetPosition);
            Constants.intakeSubsystem.runIntake(Constants.intakerollerspeed);
            isOpening = true;
            Constants.intakeSubsystem.setPivotTargetPosition(targetPosition);
        } else if (Constants.intakeSubsystem.isIntakeOpen()) {
            // Intake is open; stop the rollers and close it
            targetPosition = Constants.INTAKE_CLOSED_POSITION;
            Constants.intakeSubsystem.setPivotPosition(targetPosition);
            Constants.intakeSubsystem.stopIntake();
            Constants.intakeSubsystem.setPivotTargetPosition(targetPosition);
            isOpening = false;
        } else {
            // Intake is in between; decide action based on current position
            double currentPosition = Constants.intakeSubsystem.getPivotPosition();
            double midpoint = (Constants.INTAKE_OPEN_POSITION + Constants.INTAKE_CLOSED_POSITION) / 2.0;
            if (currentPosition < midpoint) {
                // Closer to closed position; open it
                targetPosition = Constants.INTAKE_OPEN_POSITION;
                Constants.intakeSubsystem.setPivotPosition(targetPosition);
                Constants.intakeSubsystem.runIntake(Constants.intakerollerspeed);
                Constants.intakeSubsystem.setPivotTargetPosition(targetPosition);
                isOpening = true;
            } else {
                // Closer to open position; close it
                targetPosition = Constants.INTAKE_CLOSED_POSITION;
                Constants.intakeSubsystem.setPivotPosition(targetPosition);
                Constants.intakeSubsystem.stopIntake();
                Constants.intakeSubsystem.setPivotTargetPosition(targetPosition);
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
        return Constants.intakeSubsystem.isPivotAtPosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the pivot motor to prevent further movement
        Constants.intakeSubsystem.stopPivot();
        // If the intake was closing, ensure the rollers are stopped
        if (!isOpening) {
            Constants.intakeSubsystem.stopIntake();
        }
    }
}