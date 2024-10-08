// RobotContainer.java

package frc.robot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.commands.DriveCommand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
    // Subsystems
    private final Drivetrain drivetrain = new Drivetrain();

    // Controllers
    private final XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER_PORT);

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
        // Example: Bind a button to a command
        // new JoystickButton(driverController, XboxController.Button.kA.value)
        //     .whenPressed(new SomeCommand());
    }

    public Command getAutonomousCommand() {
        // Return the autonomous command
        return null; // Replace with actual autonomous command
    }
}