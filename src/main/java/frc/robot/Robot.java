// Robot.java

package frc.robot;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;



public class Robot extends TimedRobot  {
    private Command autonomousCommand;
    private RobotContainer robotContainer;

    public void robotInit() {
        CameraServer.startAutomaticCapture("Intake Camera", 0);
        robotContainer = new RobotContainer();
        autonomousCommand = robotContainer.getAutonomousCommand();
        Constants.intakeSubsystem.resetPivotEncoder();

    }

    public void robotPeriodic() {
        Constants.drivetrain.getOdometry().update(Constants.drivetrain.getHeading(),
                                              Constants.drivetrain.getLeftEncoderPosition(),
                                              Constants.drivetrain.getRightEncoderPosition());

        // Runs the Scheduler
        CommandScheduler.getInstance().run();
        SmartDashboard.putBoolean("isBrownout", RobotController.isBrownedOut());
    }

    public void autonomousInit() {

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
    public void simulationPeriodic() {

    System.out.println("Current Odometry - X: " + Constants.drivetrain.getPose().getX() + 
                           " Y: " + Constants.drivetrain.getPose().getY() + 
                           " Rotation (Degrees): " + Constants.drivetrain.getPose().getRotation().getDegrees());

}
}