/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4737.robot;

import org.usfirst.frc.team4737.lib.oi.Gamepad;
import org.usfirst.frc.team4737.lib.oi.XboxController;
import org.usfirst.frc.team4737.robot.commands.FollowTrajectory;
import org.usfirst.frc.team4737.robot.commands.TeleopDrive;

import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.InstantCommand;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Waypoint;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	public final Gamepad driver;
	
	public OI() {
		driver = new XboxController(0);
		
		new Trigger() {
			@Override
			public boolean get() {
				return driver.getAxis("LT").get() != 0 || driver.getAxis("LT").get() != 0
						|| driver.getAxis("LS_X").get() != 0;
			}
		}.whenActive(new TeleopDrive());
		
		// TODO add autonomous testing routines to buttons
		driver.getButton("X").whenActive(new FollowTrajectory(new Waypoint[] {
				new Waypoint(0, 0, Pathfinder.d2r(0)),
				new Waypoint(5, 0, Pathfinder.d2r(0)),
		}, Robot.DRIVE.DEFAULT_TRAJ_CONFIG, 0.5));
		
		driver.getButton("A").whenActive(new InstantCommand() {
			@Override
			protected void initialize() {
				Robot.DRIVE.position.resetPos();
			}
		});
	}
	
}
