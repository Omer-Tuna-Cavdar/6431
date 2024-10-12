package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.Constants;

public class OpenintakeAutoCommand extends Command {;
    private double targetPosition;
    public OpenintakeAutoCommand(Intake intake) {
      
        addRequirements(intake);
    }

    @Override
    public void initialize() {
            targetPosition = Constants.INTAKE_OPEN_POSITION;
            }

    @Override
    public void execute() {
        Constants.intakesubsystem.setPivotPosition(targetPosition);
        Constants.intakesubsystem.runIntake(Constants.intakerollerspeed);  
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
    }
}