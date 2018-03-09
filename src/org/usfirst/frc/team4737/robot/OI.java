/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4737.robot;

import org.usfirst.frc.team4737.lib.oi.Gamepad;
import org.usfirst.frc.team4737.lib.oi.XboxController;
import org.usfirst.frc.team4737.robot.commands.TeleopDrive;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

	public final Gamepad driver;
	
	public OI() {
		driver = new XboxController(0);
		
		TeleopDrive.TRIGGER.whenActive(new TeleopDrive());
		
		// TODO add autonomous testing routines to buttons
	}
	
}
