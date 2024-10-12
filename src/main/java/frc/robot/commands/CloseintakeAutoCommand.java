package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;

public class CloseintakeAutoCommand extends Command {;
    private double targetPosition;
    private boolean isOpening;

    public CloseintakeAutoCommand(Intake intake) {
      
        addRequirements(intake);
    }

    @Override
    public void initialize() {
            // Intake is closed; open it and start the rollers
            targetPosition = Constants.INTAKE_CLOSED_POSITION;
            Constants.intakesubsystem.setPivotPosition(targetPosition);
            Constants.intakesubsystem.stopIntake();
            isOpening = true;
    }

    @Override
    public void execute() {
        // No additional actions needed; control is handled in the Intake subsystem's periodic method
    }

    @Override
    public boolean isFinished() {
        // Command finishes when the pivot reaches the target position
        return Constants.intakesubsystem.isPivotAtPosition(targetPosition);
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the pivot motor to prevent further movement
        Constants.intakesubsystem.stopPivot();
        // If the intake was closing, ensure the rollers are stopped
        if (!isOpening) {
            Constants.intakesubsystem.stopIntake();
        }
    }
}