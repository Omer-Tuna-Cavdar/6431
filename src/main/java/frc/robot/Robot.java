// Robot.java

package frc.robot;


import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



public class Robot extends TimedRobot  {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public void robotInit() {
        robotContainer = new RobotContainer();
        autonomousCommand = robotContainer.getAutonomousCommand();
        Constants.intakeSubsystem.resetPivotEncoder();

    }

    public void robotPeriodic() {
        // Runs the Scheduler
        CommandScheduler.getInstance().run();
        SmartDashboard.putBoolean("isBrownout", RobotController.isBrownedOut());
    }

    public void autonomousInit() { // Your autonomous command

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
    public void practiceInit(){
                Constants.drivetrain.zeroGyro();

    }
}