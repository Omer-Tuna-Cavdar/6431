// RobotContainer.java

package frc.robot;
import frc.robot.commands.*;
import java.util.List;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
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

        // Configure the default command for the drivetrain
        Constants.drivetrain.setDefaultCommand(
            new DriveCommand(
                Constants.drivetrain,
                () -> -driverController.getLeftY(), // Invert Y-axis
                () -> driverController.getRightX()
            )
        );

        // Configure button bindings
        configureButtonBindings();}

    
        



    private void configureButtonBindings() {
        JoystickButton r2 = new JoystickButton(driverController, 8);
        JoystickButton l1 = new JoystickButton(driverController, 5);
        JoystickButton l2 = new JoystickButton(driverController, 7);
        JoystickButton cross = new JoystickButton(driverController, 2);
        JoystickButton triangle = new JoystickButton(driverController, 4);

l2.onTrue(new InstantCommand(() -> {
    ToggleIntakeCommand.schedule();
    if (!ToggleIntakeCommand.isintakeOpening()) {
        new SequentialCommandGroup(
            new InstantCommand(() -> Constants.intakeSubsystem.runIntake(Constants.INTAKE_ROLLER_SPEED)),
            new WaitCommand(0.5), // Wait for 200 milliseconds
            new InstantCommand(() -> Constants.intakeSubsystem.stopIntake())
        ).schedule();
    }
}));
r2.onTrue(new SequentialCommandGroup(
    new InstantCommand(() -> Constants.shooterSubsystem.runShooter(Constants.SHOOTER_TARGET_RPM)),
    new WaitCommand(1.0), // Wait for 1 second
    new InstantCommand(() -> Constants.intakeSubsystem.runIntake(-Constants.INTAKE_ROLLER_SPEED))
));
l1.onTrue(shooterStop);
cross.onTrue(new InstantCommand(() -> Constants.intakeSubsystem.runIntake(Constants.INTAKE_ROLLER_SPEED)));
triangle.onTrue(new InstantCommand(() -> Constants.intakeSubsystem.runIntake(-0.7)));
    }
    public Command getAutonomousCommand() {
        // 1. Create trajectory settings
        TrajectoryConfig forwardConfig = new TrajectoryConfig(
        Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.drivetrain.getKinematics());

TrajectoryConfig backwardConfig = new TrajectoryConfig(
        Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared)
            .setKinematics(Constants.drivetrain.getKinematics())
            .setReversed(true);

// 2. Generate the backward trajectory (robot moves backward)
Trajectory backwardTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at (0, 0) facing forward (0 degrees)
    new Pose2d(0, 0, new Rotation2d(0)),
    // No interior waypoints
    List.of(),
    // End at (2.142, 0) but since it's reversed, robot moves backward to this point
    new Pose2d(2.142, 0, new Rotation2d(0)),
    backwardConfig
);
        // 2. Gen

// 3. Generate the forward trajectory (robot moves forward back to starting point)
Trajectory forwardTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at (2.142, 0) facing forward
    new Pose2d(2.142, 0, new Rotation2d(0)),
    // No interior waypoints
    List.of(),
    // End at (0, 0)
    new Pose2d(0, 0, new Rotation2d(0)),
    forwardConfig
);
Trajectory fullTrajectory = backwardTrajectory.concatenate(forwardTrajectory);


        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
        PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        RamseteCommand Goto2ndNoteRamseteCommand = new RamseteCommand(
            backwardTrajectory,
            Constants.drivetrain::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
            Constants.drivetrain.getKinematics(),
            Constants.drivetrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0), // Left PID Controller
            new PIDController(Constants.kPDriveVel, 0, 0), // Right PID Controller
            Constants.drivetrain::tankDriveVolts
        );
        RamseteCommand Gobackfrom2ndNoteRamseteCommand = new RamseteCommand(
            forwardTrajectory,
            Constants.drivetrain::getPose,
            new RamseteController(),
            new SimpleMotorFeedforward(Constants.ksVolts, Constants.kvVoltSecondsPerMeter, Constants.kaVoltSecondsSquaredPerMeter),
            Constants.drivetrain.getKinematics(),
            Constants.drivetrain::getWheelSpeeds,
            new PIDController(Constants.kPDriveVel, 0, 0), // Left PID Controller
            new PIDController(Constants.kPDriveVel, 0, 0), // Right PID Controller
            Constants.drivetrain::tankDriveVolts
        );
        


        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> Constants.drivetrain.resetOdometry(fullTrajectory.getInitialPose())),
                new InstantCommand(() -> Constants.shooterSubsystem.runShooter(Constants.SHOOTER_TARGET_RPM)),
                new WaitCommand(0.5),
                new InstantCommand(() -> Constants.intakeSubsystem.runIntake(-Constants.INTAKE_ROLLER_SPEED)),
                new InstantCommand(() -> Constants.intakeSubsystem.setPivotPosition(Constants.INTAKE_OPEN_POSITION)),
                new InstantCommand(() -> Constants.intakeSubsystem.runIntake(Constants.INTAKE_ROLLER_SPEED)),  
                Goto2ndNoteRamseteCommand,
                new InstantCommand(() -> Constants.intakeSubsystem.setPivotPosition(Constants.INTAKE_CLOSED_POSITION)),
                new InstantCommand(() -> Constants.intakeSubsystem.stopIntake()), 
                Gobackfrom2ndNoteRamseteCommand,
                new InstantCommand(() -> Constants.shooterSubsystem.runShooter(Constants.SHOOTER_TARGET_RPM)),
                new WaitCommand(0.5),
                new InstantCommand(() -> Constants.intakeSubsystem.runIntake(-Constants.INTAKE_ROLLER_SPEED)),
                new InstantCommand(() -> Constants.drivetrain.stop()));
    }

    
}