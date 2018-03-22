package org.usfirst.frc.team4737.robot.commands;

import org.usfirst.frc.team4737.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;
import jaci.pathfinder.*;

/**
 *
 */
public class FollowTrajectory extends Command {

	private Waypoint[] points;
	private Trajectory.Config config;
	private double maxSpeed;
	
    public FollowTrajectory(Waypoint[] points, Trajectory.Config config, double maxSpeed) {
        requires(Robot.DRIVE);
        this.points = points;
        this.config = config;
        this.maxSpeed = maxSpeed;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.DRIVE.setTrajectory(points, config, maxSpeed);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return !Robot.DRIVE.follower.running();
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
