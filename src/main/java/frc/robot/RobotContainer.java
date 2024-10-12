// RobotContainer.java

package frc.robot;

import frc.robot.commands.*;

import java.util.function.BooleanSupplier;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
public class RobotContainer {
    // Subsystems
    // Controllers and Buttons
    private final PS5Controller driverController = new PS5Controller(Constants.kDriverControllerPort);
    BooleanSupplier l1ButtonPressed = () -> driverController.getL1Button();
    BooleanSupplier r2ButtonPressed =() -> driverController.getR2Button();

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
       JoystickButton intakeButton = new JoystickButton(driverController,5);
        intakeButton.onTrue(ToggleIntakeCommand);
        JoystickButton shooterbutton = new JoystickButton(driverController, 8);
        shooterbutton.onTrue(shootCommand);
    }

    public Command getAutonomousCommand() {
        // Return the autonomous command
        return new PathPlannerAuto("3NoteAuto");
    // Replace with actual autonomous command
    }
}