/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4737.robot;

import org.usfirst.frc.team4737.lib.Constant;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {

	// Actuators ########################################

	// Drive talons
	public static final int TALON_LF_DRIVE = 11;
	public static final int TALON_LR_DRIVE = 12;
	public static final int TALON_RF_DRIVE = 13;
	public static final int TALON_RR_DRIVE = 14;

	// Sensors ##########################################

	// Drive encoders
	public static final int ENC_L_A = 0;
	public static final int ENC_L_B = 1;
	public static final int ENC_R_A = 2;
	public static final int ENC_R_B = 3;

	public static final int ENC_PULSES_PER_REV = 360; // E4T OEM Miniature Optical Encoder Kit (am-3132)

	// Physical Constants ###############################

	public static final double WHEEL_DIAM_FEET = (6.0 / 12.0);
	public static final double WHEELBASE_WIDTH = 24.25 / 12.0;
	
	public static final double DRIVE_MAX_SPEED = 10; // TODO find more accurate value?

	public static final double ENC_FEET_PER_PULSE = Math.PI * WHEEL_DIAM_FEET / ENC_PULSES_PER_REV;

	// Control Constants ################################

	public static final Constant<Double> DRIVE_MAX_SPEED_PCT = new Constant<>("driveMaxS", 1.0);
	public static final Constant<Double> DRIVE_MAX_ACCEL_PCT = new Constant<>("driveMaxA", 8.0);
	public static final Constant<Double> DRIVE_MAX_JERK_PCT = new Constant<>("driveMaxJ", 30.0);

//	public static final Constant<Double> ENC_LPOS_P = new Constant<>("encLPosP", 0.0);
//	public static final Constant<Double> ENC_LPOS_I = new Constant<>("encLPosI", 0.0);
//	public static final Constant<Double> ENC_LPOS_D = new Constant<>("encLPosD", 0.0);
//	public static final Constant<Double> ENC_LVEL_P = new Constant<>("encLVelP", 0.0);
//	public static final Constant<Double> ENC_LVEL_I = new Constant<>("encLVelI", 0.0);
//	public static final Constant<Double> ENC_LVEL_D = new Constant<>("encLVelD", 0.0);
//	public static final Constant<Double> ENC_LVEL_F = new Constant<>("encLVelF", 0.0);
//
//	public static final Constant<Double> ENC_RPOS_P = new Constant<>("encRPosP", 0.0);
//	public static final Constant<Double> ENC_RPOS_I = new Constant<>("encRPosI", 0.0);
//	public static final Constant<Double> ENC_RPOS_D = new Constant<>("encRPosD", 0.0);
//	public static final Constant<Double> ENC_RVEL_P = new Constant<>("encRVelP", 0.0);
//	public static final Constant<Double> ENC_RVEL_I = new Constant<>("encRVelI", 0.0);
//	public static final Constant<Double> ENC_RVEL_D = new Constant<>("encRVelD", 0.0);
//	public static final Constant<Double> ENC_RVEL_F = new Constant<>("encRVelF", 0.0);

	public static final Constant<Double> FOLLOW_kP = new Constant<>("follow_kP", 0.5);
	public static final Constant<Double> FOLLOW_kI = new Constant<>("follow_kI", 0.0);
	public static final Constant<Double> FOLLOW_kD = new Constant<>("follow_kD", 0.0);
	public static final Constant<Double> FOLLOW_kV = new Constant<>("follow_kV", 1.0 / 10.5); // 10.5 ft/s
	public static final Constant<Double> FOLLOW_kA = new Constant<>("follow_kA", 0.0);
	public static final Constant<Double> FOLLOW_kH = new Constant<>("follow_kH", 1.0 / (DRIVE_MAX_SPEED / WHEELBASE_WIDTH));
	public static final Constant<Double> FOLLOW_kHP = new Constant<>("follow_kHP", 0.66);
	public static final Constant<Double> FOLLOW_kHD = new Constant<>("follow_kHD", 0.5);

}
