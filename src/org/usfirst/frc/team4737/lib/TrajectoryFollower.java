package org.usfirst.frc.team4737.lib;

import java.util.Timer;
import java.util.TimerTask;

import org.jblas.DoubleMatrix;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import jaci.pathfinder.Trajectory;

public class TrajectoryFollower {

	private class UpdateTask extends TimerTask {

		private TrajectoryFollower tf;

		private UpdateTask(TrajectoryFollower tf) {
			this.tf = tf;
		}

		@Override
		public void run() {
			tf.update();
		}

	}

	private double kP;
	private double kI;
	private double kD;
	private double kV;
	private double kA;
	private double kH;
	private double kHP;

	private DriveDeadReckoner position;
	private DifferentialDrive drive;

	private Timer updateLoop;
	private TimerTask currentTask;

	private Trajectory traj;
	private int segment;

	private boolean running;

	/**
	 * 
	 * @param position
	 * @param leftMotor
	 * @param rightMotor
	 * @param kP
	 *            The proportional term. This is usually quite high (0.8 - 1.0 are
	 *            common values)
	 * @param kI
	 *            The integral term. Currently unused.
	 * @param kD
	 *            The derivative term. Adjust this if you are unhappy with the
	 *            tracking of the follower. *****Currently unused.
	 * @param kV
	 *            The velocity ratio. This should be 1 over your maximum velocity @
	 *            100% throttle. This converts m/s given by the algorithm to a scale
	 *            of -1..1 to be used by your motor controllers
	 * @param kH
	 *            The turning ratio. This should be 1 over your maximum turning
	 *            velocity @ 100% steer.
	 * @param kA
	 *            The acceleration term. Adjust this if you want to reach higher or
	 *            lower speeds faster.
	 */
	public TrajectoryFollower(DriveDeadReckoner position, DifferentialDrive controller, double kP, double kI, double kD,
			double kV, double kA, double kH, double kHP) {
		this.position = position;
		this.drive = controller;
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kV = kV;
		this.kA = kA;
		this.kH = kH;
		this.kHP = kHP;

		updateLoop = new Timer();

		traj = null;
		segment = 0;

		running = false;
	}

	private void update() {
		if (segment < traj.length()) {

			Trajectory.Segment seg = traj.get(segment);
			Trajectory.Segment lastSeg = segment > 1 ? traj.get(segment - 1) : seg;

			double ff = kV * seg.velocity + kA * seg.acceleration;
			double ffH = kH * (seg.heading - lastSeg.heading);

			DoubleMatrix vSeg = new DoubleMatrix(new double[] { Math.cos(seg.heading), Math.sin(seg.heading) });
			DoubleMatrix vEps = new DoubleMatrix(
					new double[] { seg.x - position.getGlobalX(), seg.y - position.getGlobalY() });

			DoubleMatrix vEpsP = vSeg.mul(vEps.dot(vSeg));
			DoubleMatrix vEpsR = vEps.sub(vEpsP);

			DoubleMatrix vSegPerp = new DoubleMatrix(new double[] { -vSeg.get(1), vSeg.get(0) });

			double leadingError = vEpsP.dot(vSeg);
			double sideError = vEpsR.dot(vSegPerp);

			double throttle = leadingError * kP + ff;
			double steer = sideError * kHP + ffH;

			drive.arcadeDrive(throttle, steer, false);
		} else {
			currentTask.cancel();
			running = false;
			// TODO replace with this.cancel() ?
		}
	}

	public void followTrajectory(Trajectory traj) {
		// Treat a null parameter the same as calling cancel()
		// Cancel old trajectory to "overwrite" it
		if (traj == null || running)
			cancel();
		if (traj.length() == 0)
			return;
		double period = traj.get(0).dt;

		segment = 0;

		updateLoop.schedule(currentTask = new UpdateTask(this), 0L, (long) (period * 1000));
		running = true;
	}

	public void cancel() {
		if (currentTask != null)
			currentTask.cancel();
		running = false;
	}

	public boolean running() {
		return running;
	}

	// TODO write test function

}
