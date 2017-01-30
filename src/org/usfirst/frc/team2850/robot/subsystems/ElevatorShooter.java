package org.usfirst.frc.team2850.robot.subsystems;

import org.usfirst.frc.team2850.robot.RobotMap;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

/**
 *
 */
public class ElevatorShooter extends PIDSubsystem {

	Spark shootInner = new Spark(RobotMap.p_shootInner);
	Spark shootOuter = new Spark(RobotMap.p_shootOuter);
	Spark elevatorMotor = new Spark(RobotMap.p_elevatorMotor);
	public static Encoder shootInnerEncoder = new Encoder(RobotMap.p_shootInnerEncoderA, RobotMap.p_shootInnerEncoderB, false, Encoder.EncodingType.k4X);
	public static Encoder shootOuterEncoder = new Encoder(RobotMap.p_shootOuterEncoderA, RobotMap.p_shootOuterEncoderB, true, Encoder.EncodingType.k4X);
	public static double innerEncInput = 0;
	private static double pShoot = .003;
	private static double iShoot = 0;
	private static double dShoot = 0;
	
	public static double outerGain = .333;
    // Initialize your subsystem here
    public ElevatorShooter() {
        // Use these to get going:
        // setSetpoint() -  Sets where the PID controller should move the system
        //                  to
        // enable() - Enables the PID controller.
    	super("ElevatorShooter", pShoot, iShoot, dShoot);
    }

    public void initDefaultCommand() {
        // Set the default command for a subsystem here.
        //setDefaultCommand(new MySpecialCommand());
    }

    protected double returnPIDInput() {
        // Return your input value for the PID loop
        // e.g. a sensor, like a potentiometer:
        // yourPot.getAverageVoltage() / kYourMaxVoltage;
    	innerEncInput = shootInnerEncoder.getRate();
        return innerEncInput;
    }

    protected void usePIDOutput(double output) {
        // Use output to drive your system, like a motor
        // e.g. yourMotor.set(output);
    	shootInner.set(output);
    	shootOuter.set(-output*outerGain);
    }
    
    public void shoot(double power) {
    	shootInner.set(power);
    	shootOuter.set(-power*outerGain);
    }
    
    public void shootStop() {
    	shootInner.set(0);
    	shootOuter.set(0);
    }
    
    public void alterOuterGain(double newGain) {
    	outerGain += newGain;
    }
    
    public void elevatorIn() {
    	elevatorMotor.set(.8);
    }
    
    public void elevatorOut() {
    	elevatorMotor.set(-.8);
    }
    
    public void elevatorSetZero() {
    	elevatorMotor.set(0);
    }
}
