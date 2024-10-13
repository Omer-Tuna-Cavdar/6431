// RobotContainer.java

package frc.robot;
import frc.robot.commands.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
public class RobotContainer {
    // Subsystems
    
    // Controllers and Buttons
    private final PS5Controller driverController = new PS5Controller(Constants.kDriverControllerPort);
    // Commands
    ToggleIntakeCommand ToggleIntakeCommand = new ToggleIntakeCommand(Constants.intakeSubsystem);
    ShootCommand shootCommand = new ShootCommand(Constants.shooterSubsytem, Constants.intakeSubsystem, Constants.shooterTargetRPM);
    //Auto 
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        // Configure the default command for the drivetrain
        Constants.drivetrain.setDefaultCommand(
            new DriveCommand(
                Constants.drivetrain,
                () -> -driverController.getLeftY(), // Invert Y-axis
                () -> driverController.getRightX()
            )
        );

        // Configure button bindings
        configureButtonBindings();

        autoChooser = AutoBuilder.buildAutoChooser();
        NamedCommands.registerCommand("openintake", ToggleIntakeCommand);
        NamedCommands.registerCommand("shootCommand", shootCommand);


        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureButtonBindings() {
        JoystickButton l1 = new JoystickButton(driverController, 4);
        JoystickButton r2 = new JoystickButton(driverController, 8);
        l1.onTrue(ToggleIntakeCommand);
        r2.onTrue(shootCommand);
    }
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("2NoteAuto");
      }
    
    
}