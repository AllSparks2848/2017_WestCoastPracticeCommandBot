package org.usfirst.frc.team2850.robot.subsystems;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import org.usfirst.frc.team2850.robot.subsystems.DriveTrain;
import com.kauailabs.navx.frc.AHRS;

/**
 *
 */
public class GyroPIDController extends PIDSubsystem {
//	sensor(s)
	public static AHRS navX = new AHRS(SPI.Port.kMXP);
	
//	navX PID variables
	private static double kP = 0.025;
	private static double kI = 0.0001;
	private static double kD = 0.06;
	public static double navXInput = 0;
	
//     Initialize subsystem here
    public GyroPIDController() {
    	super("GyroPIDController", kP, kI, kD);
    }

    public void initDefaultCommand() {
    }

    protected double returnPIDInput() {
        // Return input value for the PID loop
    	navXInput = navX.getAngle();
        return navXInput;
    }

    protected void usePIDOutput(double output) {
        // Use output to drive system
    	DriveTrain.drive1.tankDrive(output, -output);
    	DriveTrain.drive2.tankDrive(output, -output);
    }
}
