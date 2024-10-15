// Robot.java

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;



public class Robot extends TimedRobot {
    private Command autonomousCommand;
    private RobotContainer robotContainer;
    private static DoubleLogEntry xPoseLog;
    private DoubleLogEntry yPoseLog;

    public void robotInit() {
        robotContainer = new RobotContainer();
        Constants.intakeSubsystem.resetPivotEncoder();
        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        xPoseLog = new DoubleLogEntry(log, "/drivetrain/pose/x");
        yPoseLog = new DoubleLogEntry(log, "/drivetrain/pose/y");

    }

    public void robotPeriodic() {
        // Runs the Scheduler
        CommandScheduler.getInstance().run();
        SmartDashboard.putBoolean("isBrownout", RobotController.isBrownedOut());
    }

    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }
    public void autonomousPeriodic(){
    }

    public void teleopInit() {
        // Cancel autonomous when teleop starts
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }
}