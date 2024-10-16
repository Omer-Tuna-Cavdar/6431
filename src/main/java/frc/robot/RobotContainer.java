// RobotContainer.java

package frc.robot;
import frc.robot.commands.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
    // Subsystems
    
    // Controllers and Buttons
    private final PS5Controller driverController = new PS5Controller(Constants.kDriverControllerPort);
    // Commands
    ToggleIntakeCommand ToggleIntakeCommand = new ToggleIntakeCommand(Constants.intakeSubsystem);
    ShootCommand shootCommand = new ShootCommand(Constants.shooterSubsytem, Constants.intakeSubsystem, Constants.shooterTargetRPM);
    ShooterStop shooterStop = new ShooterStop(Constants.shooterSubsytem, Constants.intakeSubsystem, Constants.shooterTargetRPM);
    //Auto 
    private SendableChooser<Command> autoChooser;
    
    public RobotContainer() {
        CommandScheduler.getInstance().setPeriod(0.1);
        CommandScheduler.getInstance().registerSubsystem(Constants.intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(Constants.shooterSubsytem);
        CommandScheduler.getInstance().registerSubsystem(Constants.drivetrain);

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
        NamedCommands.registerCommand("openintake", (new InstantCommand(() -> {
    ToggleIntakeCommand.schedule();
    if (!ToggleIntakeCommand.isintakeOpening()) {
        new SequentialCommandGroup(
            new InstantCommand(() -> Constants.intakeSubsystem.runIntake(Constants.intakerollerspeed)),
            new WaitCommand(0.5), // Wait for 200 milliseconds
            new InstantCommand(() -> Constants.intakeSubsystem.stopIntake())
        ).schedule();
    }
    if(Constants.intakeSubsystem.isBumperPressed()){
        new InstantCommand(() -> Constants.intakeSubsystem.stopIntake()).schedule();
    }
})));
        NamedCommands.registerCommand("shootCommand", new SequentialCommandGroup(
    new InstantCommand(() -> Constants.shooterSubsytem.runShooter(Constants.shooterTargetRPM)),
    new WaitCommand(1.0), // Wait for 1 second
    new InstantCommand(() -> Constants.intakeSubsystem.runIntake(-Constants.intakerollerspeed))
));
        NamedCommands.registerCommand("stopshooter", shooterStop);


        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureButtonBindings() {
        JoystickButton r2 = new JoystickButton(driverController, 8);
        JoystickButton intakeButton = new JoystickButton(driverController, 5);
        JoystickButton l2 = new JoystickButton(driverController, 7);
intakeButton.onTrue(new InstantCommand(() -> {
    ToggleIntakeCommand.schedule();
    if (!ToggleIntakeCommand.isintakeOpening()) {
        new SequentialCommandGroup(
            new InstantCommand(() -> Constants.intakeSubsystem.runIntake(Constants.intakerollerspeed)),
            new WaitCommand(0.5), // Wait for 200 milliseconds
            new InstantCommand(() -> Constants.intakeSubsystem.stopIntake())
        ).schedule();
    }
}));
r2.onTrue(new SequentialCommandGroup(
    new InstantCommand(() -> Constants.shooterSubsytem.runShooter(Constants.shooterTargetRPM)),
    new WaitCommand(1.0), // Wait for 1 second
    new InstantCommand(() -> Constants.intakeSubsystem.runIntake(-Constants.intakerollerspeed))
));
l2.onTrue(shooterStop);
    }
    public Command getAutonomousCommand() {
        return new PathPlannerAuto("2NoteAuto");
      }
      public Pose2d getStaringPoseFromAuto(){
    return PathPlannerAuto.getStaringPoseFromAutoFile("2NoteAuto");
      }
    
}