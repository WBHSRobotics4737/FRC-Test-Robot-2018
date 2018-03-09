package org.usfirst.frc.team4737.lib;

import edu.wpi.first.wpilibj.Preferences;

/**
 * A class for interfacing with {@link edu.wpi.first.wpilibj.Preferences WPILib
 * Preferences}. Values are automatically saved by NetworkTables and are
 * viewable through the SmartDashboard.
 *
 * @param <T>
 *            The object type of the constant. This can be a Boolean, Double,
 *            Float, Integer, Long, or String.
 */
public final class Constant<T> {

	private String key;
	private T value;

	@SuppressWarnings("unchecked")
	public Constant(String key, T backup) {
		this.key = key;
		if (backup instanceof Boolean)
			value = (T) (Object) Preferences.getInstance().getBoolean(key, (Boolean) backup);
		else if (backup instanceof Double)
			value = (T) (Object) Preferences.getInstance().getDouble(key, (Double) backup);
		else if (backup instanceof Float)
			value = (T) (Object) Preferences.getInstance().getFloat(key, (Float) backup);
		else if (backup instanceof Integer)
			value = (T) (Object) Preferences.getInstance().getInt(key, (Integer) backup);
		else if (backup instanceof Long)
			value = (T) (Object) Preferences.getInstance().getLong(key, (Long) backup);
		else if (backup instanceof String)
			value = (T) (Object) Preferences.getInstance().getString(key, (String) backup);
	}

	public String getKey() {
		return key;
	}

	public T val() {
		return value;
	}

	public void set(T value) {
		this.value = value;
		if (value instanceof Boolean)
			Preferences.getInstance().putBoolean(key, (Boolean) value);
		else if (value instanceof Double)
			Preferences.getInstance().putDouble(key, (Double) value);
		else if (value instanceof Float)
			Preferences.getInstance().putFloat(key, (Float) value);
		else if (value instanceof Integer)
			Preferences.getInstance().putInt(key, (Integer) value);
		else if (value instanceof Long)
			Preferences.getInstance().putLong(key, (Long) value);
		else if (value instanceof String)
			Preferences.getInstance().putString(key, (String) value);
	}

}