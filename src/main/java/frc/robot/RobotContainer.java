// RobotContainer.java

package frc.robot;
import frc.robot.commands.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RobotContainer {
    // Subsystems
    
    // Controllers and Buttons
    private final PS5Controller driverController = new PS5Controller(Constants.kDriverControllerPort);
    // Commands
    ToggleIntakeCommand ToggleIntakeCommand = new ToggleIntakeCommand(Constants.intakeSubsystem);
    ShootCommand shootCommand = new ShootCommand(Constants.shooterSubsystem, Constants.intakeSubsystem, Constants.SHOOTER_TARGET_RPM);
    ShooterStop shooterStop = new ShooterStop(Constants.shooterSubsystem, Constants.intakeSubsystem, Constants.SHOOTER_TARGET_RPM);
    //Auto 
    
    public RobotContainer() {
        CommandScheduler.getInstance().setPeriod(0.02);
        CommandScheduler.getInstance().registerSubsystem(Constants.intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(Constants.shooterSubsystem);
        CommandScheduler.getInstance().registerSubsystem(Constants.drivetrain);
        NamedCommands.registerCommand("ToggleIntake",ToggleIntakeCommand);
        NamedCommands.registerCommand("Shoot",shootCommand);

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
    }

    private void configureButtonBindings() {
        JoystickButton r2 = new JoystickButton(driverController, 8);
        JoystickButton l1 = new JoystickButton(driverController, 5);
        JoystickButton l2 = new JoystickButton(driverController, 7);
        JoystickButton cross = new JoystickButton(driverController, 2);
        JoystickButton triangle = new JoystickButton(driverController, 4);

        l2.onTrue(new InstantCommand(() -> {
            System.out.println("L2 pressed: Toggle Intake Command triggered.");
            ToggleIntakeCommand.schedule();
            if (!ToggleIntakeCommand.isintakeOpening()) {
                new SequentialCommandGroup(
                    new InstantCommand(() -> {
                        System.out.println("Running intake at speed: " + Constants.INTAKE_ROLLER_SPEED);
                        Constants.intakeSubsystem.runIntake(Constants.INTAKE_ROLLER_SPEED);
                    }),
                    new WaitCommand(0.5),
                    new InstantCommand(() -> {
                        System.out.println("Stopping intake.");
                        Constants.intakeSubsystem.stopIntake();
                    })
                ).schedule();
            }
        }));

        r2.onTrue(new SequentialCommandGroup(
            new InstantCommand(() -> {
                System.out.println("R2 pressed: Shooter running at RPM: " + Constants.SHOOTER_TARGET_RPM);
                Constants.shooterSubsystem.runShooter(Constants.SHOOTER_TARGET_RPM);
            }),
            new WaitCommand(1.0),
            new InstantCommand(() -> {
                System.out.println("Running intake in reverse.");
                Constants.intakeSubsystem.runIntake(-Constants.INTAKE_ROLLER_SPEED);
            })
        ));

        l1.onTrue(new InstantCommand(() -> {
            System.out.println("L1 pressed: Shooter stopped.");
            shooterStop.schedule();
        }));

        cross.onTrue(new InstantCommand(() -> {
            System.out.println("Cross button pressed: Running intake forward.");
            Constants.intakeSubsystem.runIntake(Constants.INTAKE_ROLLER_SPEED);
        }));

        triangle.onTrue(new InstantCommand(() -> {
            System.out.println("Triangle button pressed: Running intake reverse at speed -0.7.");
            Constants.intakeSubsystem.runIntake(-0.7);
        }));
    }

    public Command getAutonomousCommand() {
      
        Constants.drivetrain.resetOdometry(PathPlannerAuto.getStaringPoseFromAutoFile("2NoteAuto"));
        return 
        new SequentialCommandGroup(
        new ShootCommand(Constants.shooterSubsystem, Constants.intakeSubsystem, Constants.SHOOTER_TARGET_RPM),
        new ParallelCommandGroup(new ShootCommand(Constants.shooterSubsystem, Constants.intakeSubsystem, Constants.SHOOTER_TARGET_RPM),new InstantCommand(() -> {
                System.out.println("Running intake in reverse.");
                Constants.intakeSubsystem.runIntake(-Constants.INTAKE_ROLLER_SPEED);
            }))
        );
    }
}
