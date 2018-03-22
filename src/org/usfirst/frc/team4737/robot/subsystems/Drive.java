package org.usfirst.frc.team4737.robot.subsystems;

import org.usfirst.frc.team4737.lib.DriveDeadReckoner;
import org.usfirst.frc.team4737.lib.JerkLimitedTalonSRXController;
import org.usfirst.frc.team4737.lib.LazyWPITalonSRX;
import org.usfirst.frc.team4737.lib.TrajectoryFollower;
import org.usfirst.frc.team4737.robot.RobotMap;
import org.usfirst.frc.team4737.robot.commands.TeleopDrive;

import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.FitMethod;

import static org.usfirst.frc.team4737.robot.RobotMap.*;

/**
 *
 */
public class Drive extends Subsystem {

	public final Trajectory.Config DEFAULT_TRAJ_CONFIG = new Trajectory.Config(FitMethod.HERMITE_CUBIC,
			Trajectory.Config.SAMPLES_FAST, 0.01, RobotMap.DRIVE_MAX_SPEED, 10, 60);

	// Talons and sensors

	private WPI_TalonSRX lfTalon;
	private WPI_TalonSRX lrTalon;
	private WPI_TalonSRX rfTalon;
	private WPI_TalonSRX rrTalon;
	private WPI_TalonSRX[] talons;

	private Encoder lEnc;
	private Encoder rEnc;

	private AHRS navX;

	// Controllers

	// private PIDController lPosControl;
	// private PIDController rPosControl;
	// private PIDController lVelControl;
	// private PIDController rVelControl;

	private JerkLimitedTalonSRXController lSmoothDrive;
	private JerkLimitedTalonSRXController rSmoothDrive;
	private DifferentialDrive smoothDrive;
	private DifferentialDrive rawDrive;

	public TrajectoryFollower follower;

	// Other

	public DriveDeadReckoner position;

	public Drive() {
		// Initialize talons
		lfTalon = createDriveTalon(TALON_LF_DRIVE);
		lrTalon = createSlaveTalon(TALON_LR_DRIVE, lfTalon);
		rfTalon = createDriveTalon(TALON_RF_DRIVE);
		rrTalon = createSlaveTalon(TALON_RR_DRIVE, rfTalon);
		talons = new WPI_TalonSRX[] { lfTalon, lrTalon, rfTalon, rrTalon };

		//		rfTalon.setInverted(true);
		// rrTalon.setInverted(true); // Shouldn't need to invert slave talon

		// Initialize sensors
		lEnc = new Encoder(ENC_L_A, ENC_L_B, true);
		lEnc.setDistancePerPulse(ENC_FEET_PER_PULSE);
		rEnc = new Encoder(ENC_R_A, ENC_R_B, false);
		rEnc.setDistancePerPulse(ENC_FEET_PER_PULSE);

		navX = new AHRS(Port.kMXP, (byte) 200);

		// Initialize PID controls
		// double pidPeriod = 10 / 1000.; // 10 ms

		// PIDSourceOutput lControlBridge = new PIDSourceOutput(PIDSourceType.kRate);
		// lPosControl = new PIDController(ENC_LPOS_P.val(), ENC_LPOS_I.val(),
		// ENC_LPOS_D.val(), lEnc, lControlBridge,
		// pidPeriod);
		// lVelControl = new PIDController(ENC_LVEL_P.val(), ENC_LVEL_I.val(),
		// ENC_LVEL_D.val(), ENC_LVEL_F.val(),
		// lControlBridge, lfTalon, pidPeriod);

		// lEnc.setPIDSourceType(PIDSourceType.kRate);
		// lVelControl = new PIDController(ENC_LVEL_P.val(), ENC_LVEL_I.val(),
		// ENC_LVEL_D.val(), ENC_LVEL_F.val(),
		// lEnc, lfTalon, pidPeriod);

		// PIDSourceOutput rControlBridge = new PIDSourceOutput(PIDSourceType.kRate);
		// rPosControl = new PIDController(ENC_RPOS_P.val(), ENC_RPOS_I.val(),
		// ENC_RPOS_D.val(), lEnc, rControlBridge,
		// pidPeriod);
		// rVelControl = new PIDController(ENC_RVEL_P.val(), ENC_RVEL_I.val(),
		// ENC_RVEL_D.val(), ENC_RVEL_F.val(),
		// rControlBridge, rfTalon, pidPeriod);

		// rEnc.setPIDSourceType(PIDSourceType.kRate);
		// rVelControl = new PIDController(ENC_RVEL_P.val(), ENC_RVEL_I.val(),
		// ENC_RVEL_D.val(), ENC_RVEL_F.val(),
		// rEnc, rfTalon, pidPeriod);

		smoothDrive = new DifferentialDrive(
				lSmoothDrive = new JerkLimitedTalonSRXController(lfTalon, DRIVE_MAX_SPEED_PCT.val(),
						DRIVE_MAX_ACCEL_PCT.val(), DRIVE_MAX_JERK_PCT.val()),
				rSmoothDrive = new JerkLimitedTalonSRXController(rfTalon, DRIVE_MAX_SPEED_PCT.val(),
						DRIVE_MAX_ACCEL_PCT.val(), DRIVE_MAX_JERK_PCT.val()));
		lSmoothDrive.disable();
		rSmoothDrive.disable();
		smoothDrive.setSafetyEnabled(false);
		rawDrive = new DifferentialDrive(lfTalon, rfTalon);
		rawDrive.setSafetyEnabled(false);

		position = new DriveDeadReckoner(lEnc, rEnc, navX, 0.005);

		follower = new TrajectoryFollower(position, lfTalon, rfTalon, FOLLOW_kP.val(), FOLLOW_kI.val(), FOLLOW_kD.val(),
				FOLLOW_kV.val(), FOLLOW_kA.val(), FOLLOW_kH.val(), FOLLOW_kHP.val(), FOLLOW_kHD.val());
	}

	private WPI_TalonSRX createDriveTalon(int id) {
		WPI_TalonSRX talon = new LazyWPITalonSRX(id);
		talon.configVoltageCompSaturation(12, 100);
		talon.enableVoltageCompensation(true);
		talon.setNeutralMode(NeutralMode.Brake);
		talon.setSafetyEnabled(false);
		return talon;
	}

	private WPI_TalonSRX createSlaveTalon(int id, IMotorController master) {
		WPI_TalonSRX talon = createDriveTalon(id);
		talon.follow(master);
		return talon;
	}

	public void initDefaultCommand() {
		setDefaultCommand(new TeleopDrive());
	}

	double lastPrintTime = 0;

	@Override
	public void periodic() {

		if (Timer.getFPGATimestamp() - lastPrintTime >= 0.2) {
			//		SmartDashboard.putNumber("speed", (lEnc.getRate() + rEnc.getRate()) / 2.0);
			//		SmartDashboard.putNumber("omega", navX.getRate());
			SmartDashboard.putNumber("gx", position.getX());
			SmartDashboard.putNumber("gy", position.getY());
			SmartDashboard.putNumber("dir", position.getHeading());
			lastPrintTime = Timer.getFPGATimestamp();
		}

	}

	public void setTrajectory(Waypoint[] points, Trajectory.Config config, double maxSpeed) {
		config.max_velocity = maxSpeed;
		DriverStation.reportWarning("Generating trajectory...", false);
		setTrajectory(Pathfinder.generate(points, config));
		DriverStation.reportWarning("Finished.", false);
	}

	public void setTrajectory(Trajectory traj) {
		// Disable smoothers so we have control of the talons
		lSmoothDrive.pauseUpdates();
		rSmoothDrive.pauseUpdates();

		follower.followTrajectory(traj);
	}

	public void cancelTrajectory() {
		follower.cancel();
	}

	public void curvatureDrive(double speed, double rotation, boolean isQuickTurn) {
		cancelTrajectory();

		smoothDrive.curvatureDrive(speed, rotation, isQuickTurn);
	}

	// public void rawTankDrive(double leftSpeed, double rightSpeed) {
	// cancelTrajectory();
	// // Disable smoothers so we can run raw
	// lSmoothDrive.pauseUpdates();
	// rSmoothDrive.pauseUpdates();
	//
	// rawDrive.tankDrive(leftSpeed, rightSpeed, false);
	// }

	public void setRelaxed(boolean relax) {
		for (WPI_TalonSRX talon : talons)
			talon.setNeutralMode(relax ? NeutralMode.Coast : NeutralMode.Brake);
	}

}
