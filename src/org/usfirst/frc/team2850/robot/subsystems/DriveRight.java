package org.usfirst.frc.team2850.robot.subsystems;

import org.usfirst.frc.team2850.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class DriveRight extends PIDSubsystem {
	Spark rightDrive1 = new Spark(RobotMap.p_rightDrive1);
	Spark rightDrive2 = new Spark(RobotMap.p_rightDrive2);
	Spark rightDrive3 = new Spark(RobotMap.p_rightDrive3);
	
	private static double pDrive = .07;
	private static double iDrive = 0;
	private static double dDrive = .007;
	
	public static Encoder rightEncoder = new Encoder(RobotMap.p_rightEncoderA, RobotMap.p_rightEncoderB, false, Encoder.EncodingType.k4X);
	
    // Initialize your subsystem here
    public DriveRight() {
        super("DriveRight", pDrive, iDrive, dDrive);
    	rightEncoder.setDistancePerPulse(0.0117);
    	enable();
    }

    public void initDefaultCommand() {
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
        return rightEncoder.getDistance();
    }

    protected void usePIDOutput(double output) {
    	rightDrive1.set(output);
    	rightDrive2.set(output);
		rightDrive3.set(output);
    }
}
