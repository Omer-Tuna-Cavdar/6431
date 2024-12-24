// RobotContainer.java

package frc.robot;
import frc.robot.commands.*;
import java.util.List;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPRamseteController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

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

        // Configure the default command for the drivetrain
        Constants.drivetrain.setDefaultCommand(
            new DriveCommand(
                Constants.drivetrain,
                () -> -driverController.getLeftY()*0.8, // Invert Y-axis
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
        // Load the PathPlanner trajectories
        PathPlannerPath trajectoryToIntake = PathPlannerPath.fromPathFile("1");
        PathPlannerPath trajectoryToShoot = PathPlannerPath.fromPathFile("2");

        // Create commands to follow the paths

        // Sequence the autonomous commands
        return new SequentialCommandGroup(
            // Step 1: Shoot the preloaded note
            new InstantCommand(() -> {
                System.out.println("Autonomous Step 1: Starting shooter.");
                Constants.shooterSubsystem.runShooter(Constants.SHOOTER_TARGET_RPM);
            }),
            new WaitCommand(0.5), // Allow time for shooter to spin up
            new InstantCommand(() -> {
                System.out.println("Autonomous Step 1: Running intake in reverse to shoot.");
                Constants.intakeSubsystem.runIntake(-Constants.INTAKE_ROLLER_SPEED); // Run intake in reverse
            }),
            new WaitCommand(0.5), // Time for note to exit
            new InstantCommand(() -> {
                System.out.println("Autonomous Step 1: Stopping shooter and intake.");
                Constants.intakeSubsystem.stopIntake();
                Constants.shooterSubsystem.stopShooter();
            }),

            // Step 2: Open intake and collect the second note
            new InstantCommand(() -> {
                System.out.println("Autonomous Step 2: Setting pivot to open position.");
                Constants.intakeSubsystem.setPivotTargetPosition(Constants.INTAKE_OPEN_POSITION);
            }),
            new WaitUntilCommand(() -> Constants.intakeSubsystem.isIntakeOpen()), // Wait for pivot to reach the open position
            new InstantCommand(() -> {
                System.out.println("Autonomous Step 2: Starting intake roller.");
                Constants.intakeSubsystem.runIntake(Constants.INTAKE_ROLLER_SPEED); // Start intake roller
            }),
            new WaitCommand(2.0), // Ensure time for note to be collected
            new InstantCommand(() -> {
                System.out.println("Autonomous Step 2: Stopping intake roller and closing pivot.");
                Constants.intakeSubsystem.stopIntake();
                Constants.intakeSubsystem.setPivotTargetPosition(Constants.INTAKE_CLOSED_POSITION); // Set pivot to closed position
            }),
            new WaitUntilCommand(() -> Constants.intakeSubsystem.isIntakeClosed()), // Wait for pivot to reach the closed position

            // Step 3: Drive back to the shooting position//
            new InstantCommand(() -> {
                System.out.println("Autonomous Step 3: Starting shooter for second note.");
                Constants.shooterSubsystem.runShooter(Constants.SHOOTER_TARGET_RPM);
            }),
            new WaitCommand(0.5), // Allow time for shooter to spin up
            new InstantCommand(() -> {
                System.out.println("Autonomous Step 3: Running intake in reverse to shoot.");
                Constants.intakeSubsystem.runIntake(-Constants.INTAKE_ROLLER_SPEED); // Run intake in reverse to shoot
            }),
            new WaitCommand(0.5), // Time for second note to exit
            new InstantCommand(() -> {
                System.out.println("Autonomous Step 3: Stopping shooter and intake.");
                Constants.intakeSubsystem.stopIntake();
                Constants.shooterSubsystem.stopShooter();
            })
        );
    }
}
