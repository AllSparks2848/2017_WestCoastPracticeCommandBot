package org.usfirst.frc.team2850.robot;

import org.spectrum3847.RIOdroid.RIOdroid;
import org.usfirst.frc.team2850.robot.commands.AutonGearLeft;
import org.usfirst.frc.team2850.robot.subsystems.DriveTrain;
import org.usfirst.frc.team2850.robot.vision.TestUpdateReceiver;
import org.usfirst.frc.team2850.robot.vision.VisionServer;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	public static OI oi;
	public static Logger logger;
	public static FileIO fileIO = new FileIO();
    private int LOGGER_LEVEL = 5;
    boolean useConsole = true, useFile = true;
	public static RobotMap robot = new RobotMap();
	public static VisionServer visionServer;
	public static TestUpdateReceiver testUpdateReceiver;

	Command autonomousCommand;

	public static DriveTrain drivetrain = new DriveTrain();
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		logger = new Logger(useConsole, useFile, LOGGER_LEVEL);
		autonomousCommand = new AutonGearLeft();
		oi = new OI();
		SmartDashboard.putData(Scheduler.getInstance());
		SmartDashboard.putData(drivetrain);
		 RIOdroid.initUSB();
	        
	        visionServer = VisionServer.getInstance();
	        testUpdateReceiver = new TestUpdateReceiver();
	        visionServer.addVisionUpdateReceiver(testUpdateReceiver);
		
		
	}

	/**
	 * This function is called once each time the robot enters Disabled mode.
	 * You can use it to reset any subsystem information you want to clear when
	 * the robot is disabled.
	 */
	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void autonomousInit() {
		// schedule the autonomous command (example)
			autonomousCommand.start();
			
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		Scheduler.getInstance().run();
	}

	@Override
	public void teleopInit() {
			autonomousCommand.cancel();
	}

	@Override
	public void teleopPeriodic() {
		Scheduler.getInstance().run();
	}
	
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
