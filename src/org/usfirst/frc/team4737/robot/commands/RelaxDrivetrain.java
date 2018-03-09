package org.usfirst.frc.team4737.robot.commands;

import org.usfirst.frc.team4737.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RelaxDrivetrain extends Command {

	public RelaxDrivetrain() {
		requires(Robot.DRIVE);
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		Robot.DRIVE.setRelaxed(true);
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		return !Robot.getInstance().isDisabled();
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.DRIVE.setRelaxed(false);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}

}
