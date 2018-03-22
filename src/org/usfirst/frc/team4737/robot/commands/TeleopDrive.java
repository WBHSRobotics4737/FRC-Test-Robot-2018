package org.usfirst.frc.team4737.robot.commands;

import org.usfirst.frc.team4737.robot.Robot;

import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class TeleopDrive extends Command {

	public TeleopDrive() {
		requires(Robot.DRIVE);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		double throttle = Robot.OI.driver.getAxis("RT").get() - Robot.OI.driver.getAxis("LT").get();
		double rotation = Robot.OI.driver.getAxis("LS_X").get();
		boolean slowDrive = Robot.OI.driver.getButton("RB").get();
		boolean quickTurn = Robot.OI.driver.getButton("LB").get();
		
		if (slowDrive) {
			throttle *= 0.3;
			rotation *= 0.3;
		}
		
		Robot.DRIVE.curvatureDrive(throttle, rotation, quickTurn);
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return false;
	}

	// Called once after isFinished returns true
	protected void end() {
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
	}

}
