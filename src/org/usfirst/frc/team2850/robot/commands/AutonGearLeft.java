package org.usfirst.frc.team2850.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;

/**
 *
 */
public class AutonGearLeft extends CommandGroup {

    public AutonGearLeft() {
    	//addSequential(new DriveToDistance(39.0));
    	addSequential(new DriveToDistance(68));
    	addSequential(new GyroTurn(64.57));
    	addSequential(new DriveToDistance(71));
    	
    	
        // Add Commands here:
        // e.g. addSequential(new Command1());
        //      addSequential(new Command2());
        // these will run in order.

        // To run multiple commands at the same time,
        // use addParallel()
        // e.g. addParallel(new Command1());
        //      addSequential(new Command2());
        // Command1 and Command2 will run in parallel.

        // A command group will require all of the subsystems that each member
        // would require.
        // e.g. if Command1 requires chassis, and Command2 requires arm,
        // a CommandGroup containing them would require both the chassis and the
        // arm.
    }
}
