package org.usfirst.frc.team4737.lib;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Timer;
import java.util.TimerTask;

import org.jblas.DoubleMatrix;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.FitMethod;

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
	private double kHD;

	private DriveDeadReckoner position;
	private SpeedController left;
	private SpeedController right;

	private Timer updateLoop;
	private TimerTask currentTask;

	private Trajectory traj;
	private int segment;

	private boolean running;
	
	private double lastSideError;

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
	public TrajectoryFollower(DriveDeadReckoner position, SpeedController left, SpeedController right, double kP,
			double kI, double kD, double kV, double kA, double kH, double kHP, double kHD) {
		this.position = position;
		this.left = left;
		this.right = right;
		this.kP = kP;
		this.kI = kI;
		this.kD = kD;
		this.kV = kV;
		this.kA = kA;
		this.kH = kH;
		this.kHP = kHP;
		this.kHD = kHD;

		updateLoop = new Timer();

		traj = null;
		segment = 0;

		running = false;
	}

	double ff;
	double ffH;
	double leadingError;
	double sideError;

	private void update() {
		if (segment < traj.length()) {

			Trajectory.Segment seg = traj.get(segment);
			Trajectory.Segment lastSeg = segment > 1 ? traj.get(segment - 1) : seg;

			ff = kV * seg.velocity + kA * seg.acceleration;
			ffH = -kH * angleDiff(seg.heading, lastSeg.heading) / seg.dt;

			DoubleMatrix vSeg = new DoubleMatrix(new double[] { Math.cos(seg.heading), Math.sin(seg.heading) });
			DoubleMatrix vEps = new DoubleMatrix(
					new double[] { seg.x - position.getX(), seg.y - position.getY() });

			DoubleMatrix vEpsP = vSeg.mul(vEps.dot(vSeg));
			DoubleMatrix vEpsR = vEps.sub(vEpsP);

			DoubleMatrix vSegPerp = new DoubleMatrix(new double[] { -vSeg.get(1), vSeg.get(0) });

			leadingError = vEpsP.dot(vSeg);
			sideError = -vEpsR.dot(vSegPerp);

			double throttle = leadingError * kP + ff;
			double steer = ffH + sideError * kHP + ((sideError - lastSideError) / seg.dt) * kHD;

			arcadeDrive(throttle, steer);
			segment++;
			
			lastSideError = sideError;
		} else {
			cancel();
		}
	}

	private double angleDiff(double a1, double a2) {
		double a = a1 - a2;
		while (a >= Math.PI)
			a -= Math.PI * 2;
		while (a < -Math.PI)
			a += Math.PI * 2;
		return a;
	}

	private void arcadeDrive(double throttle, double steer) {
		double max = 0.5;
		throttle = Math.max(Math.min(throttle, max), -max);
		steer = Math.max(Math.min(steer, max), -max);

		double leftMotorOutput;
		double rightMotorOutput;

		double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(steer)), throttle);

		if (throttle >= 0.0) {
			// First quadrant, else second quadrant
			if (steer >= 0.0) {
				leftMotorOutput = maxInput;
				rightMotorOutput = throttle - steer;
			} else {
				leftMotorOutput = throttle + steer;
				rightMotorOutput = maxInput;
			}
		} else {
			// Third quadrant, else fourth quadrant
			if (steer >= 0.0) {
				leftMotorOutput = throttle + steer;
				rightMotorOutput = maxInput;
			} else {
				leftMotorOutput = maxInput;
				rightMotorOutput = throttle - steer;
			}
		}

		left.set(Math.max(Math.min(leftMotorOutput, max), -max));
		right.set(Math.max(Math.min(rightMotorOutput, max), -max));
	}

	public void followTrajectory(Trajectory traj) {
		// Treat a null parameter the same as calling cancel()
		// Cancel old trajectory to "overwrite" it
		if (traj == null || running)
			cancel();
		if (traj.length() == 0)
			return;
		double period = traj.get(0).dt;

		this.traj = traj;
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

	public static void main(String[] args) {
		System.out.println("Running TrajectoryFollower Test...");
		try {
			FileWriter writer = new FileWriter("testfiles/trajfollow.csv");

			double period = 0.01;

			double wheelbase = 28.0 / 12.0;
			double maxSpeed = 9.0;

			DriveDeadReckoner ddr = new DriveDeadReckoner(null, null, null, period);
			ddr.updateLoop.cancel();

			SpeedController driveL;
			SpeedController driveR;

			double kP = 0.5;
			double kV = 1.0 / maxSpeed;
			double kA = 0.0;
			double kH = 1.0 / (maxSpeed / (wheelbase));
			double kHP = .66;
			double kHD = .5;
			TrajectoryFollower tf = new TrajectoryFollower(ddr, driveL = new SpeedController() {
				public void pidWrite(double output) {
				}

				public void stopMotor() {
				}

				public void setInverted(boolean isInverted) {
				}

				double val = 0;

				public void set(double speed) {
					val = speed;
				}

				public boolean getInverted() {
					return false;
				}

				public double get() {
					return val;
				}

				public void disable() {
				}
			}, driveR = new SpeedController() {
				public void pidWrite(double output) {
				}

				public void stopMotor() {
				}

				public void setInverted(boolean isInverted) {
				}

				double val = 0;

				public void set(double speed) {
					val = speed;
				}

				public boolean getInverted() {
					return false;
				}

				public double get() {
					return val;
				}

				public void disable() {
				}
			}, kP, 0, 0, kV, kA, kH, kHP, kHD);

			Waypoint[] points = new Waypoint[] { new Waypoint(0, 0, Pathfinder.d2r(0)),
					new Waypoint(5, 2, Pathfinder.d2r(45)), new Waypoint(5, 5, Pathfinder.d2r(60)),
					new Waypoint(13, 5, Pathfinder.d2r(0)) };

			Trajectory.Config config = new Trajectory.Config(FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
					period, maxSpeed * 0.5, 10, 60);

			Trajectory traj = Pathfinder.generate(points, config);

			tf.followTrajectory(traj);
			tf.currentTask.cancel();

			writer.write("time,x,y,dataset,ffH,ff,leadE,sideE\n");

			double heading = Math.PI / 2;
			for (double t = 0; t < traj.length() * period; t += period) {
				tf.update();

				double dl = driveL.get() * maxSpeed * period;
				double dr = driveR.get() * maxSpeed * period;
				double da = (dl - dr) / wheelbase;
				heading += da;
				ddr.calculate(period, dl, dr, da, heading, 0);

				writer.write(String.format("%f,%f,%f,%s,%f,%f,%f,%f\n", t, traj.get(tf.segment - 1).x,
						traj.get(tf.segment - 1).y, "traj", tf.ffH, tf.ff, tf.leadingError, tf.sideError));
				writer.write(String.format("%f,%f,%f,%s,%f,%f,%f,%f\n", t, ddr.x, ddr.y, "pos", tf.ffH, tf.ff,
						tf.leadingError, tf.sideError));
			}

			writer.close();
		} catch (IOException e) {
			e.printStackTrace();
		}
		System.out.println("Done.");
		System.exit(0);
	}

}
