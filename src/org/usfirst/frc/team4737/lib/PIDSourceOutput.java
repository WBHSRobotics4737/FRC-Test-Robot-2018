package org.usfirst.frc.team4737.lib;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDSourceOutput implements PIDOutput, PIDSource {

	private double value;
	private PIDSourceType sourceType;

	public PIDSourceOutput(PIDSourceType type) {
		this.sourceType = type;
	}

	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {
		sourceType = pidSource;
	}

	@Override
	public PIDSourceType getPIDSourceType() {
		return sourceType;
	}

	@Override
	public double pidGet() {
		return value;
	}

	@Override
	public void pidWrite(double output) {
		value = output;
	}

}
