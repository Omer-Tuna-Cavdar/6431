// RobotContainer.java

package frc.robot;

import frc.robot.commands.*;

import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
public class RobotContainer {
    // Subsystems
    // Controllers and Buttons
    private final PS5Controller driverController = new PS5Controller(Constants.kDriverControllerPort);
    BooleanSupplier l1ButtonPressed = () -> driverController.getL1Button();
    BooleanSupplier r2ButtonPressed =() -> driverController.getR2Button();
    private final Trigger L1 = new Trigger(l1ButtonPressed);
    private final Trigger R2 = new Trigger(r2ButtonPressed);
    // Commands
    ToggleIntakeCommand ToggleIntakeCommand = new ToggleIntakeCommand(Constants.intakesubsystem);
    ShootCommand shootCommand = new ShootCommand(Constants.shotersubsystem, Constants.intakesubsystem, Constants.shooterTargetRPM);


    public RobotContainer() {
        // Configure the default command for the drivetrain
        Constants.drivetrainsubsystem.setDefaultCommand(
            new DriveCommand(
                Constants.drivetrainsubsystem,
                () -> -driverController.getLeftY(), // Invert Y-axis
                () -> driverController.getRightX()
            )
        );

        // Configure button bindings
        configureButtonBindings();
    }

    private void configureButtonBindings() {
        L1.debounce(0.1).onTrue(ToggleIntakeCommand);
        R2.debounce(0.1).onTrue(shootCommand);
    }

    public Command getAutonomousCommand() {
        // Return the autonomous command
        return null; // Replace with actual autonomous command
    }
}