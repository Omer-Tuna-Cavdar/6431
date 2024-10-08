// RobotContainer.java

package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.*;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
public class RobotContainer {
    // Subsystems
    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intakeSubsystem = new Intake();
    // Controllers and Buttons
    private final PS5Controller driverController = new PS5Controller(Constants.kDriverControllerPort);
    BooleanSupplier l1ButtonPressed = () -> driverController.getL1Button();
    private final Trigger L1 = new Trigger(l1ButtonPressed);
    // Commands
    ToggleIntakeCommand ToggleIntakeCommand = new ToggleIntakeCommand(intakeSubsystem);
    


    public RobotContainer() {
        // Configure the default command for the drivetrain
        drivetrain.setDefaultCommand(
            new DriveCommand(
                drivetrain,
                () -> -driverController.getLeftY(), // Invert Y-axis
                () -> driverController.getRightX()
            )
        );

        // Configure button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        L1.debounce(0.1).onTrue(ToggleIntakeCommand);
        
    }

    public Command getAutonomousCommand() {
        // Return the autonomous command
        return null; // Replace with actual autonomous command
    }
}