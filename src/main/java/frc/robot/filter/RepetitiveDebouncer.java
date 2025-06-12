package frc.robot.filter;

public class RepetitiveDebouncer {
	private int repetitions;
	private final int repetitionThreshold;
	private boolean desiredValue;

	public RepetitiveDebouncer(int repetitionThreshold) {
		repetitions = 0;
		this.repetitionThreshold = repetitionThreshold;
		desiredValue = true;
	}
	public RepetitiveDebouncer(int repetitionThreshold, boolean desiredValue) {
		repetitions = 0;
		this.repetitionThreshold = repetitionThreshold;
		this.desiredValue = desiredValue;
	}

	public void addValue(boolean value) {
		if (value == desiredValue) {
			repetitions++;
		} else {
			repetitions = 0;
		}
	}

	public boolean getBoolean() {
		return repetitions>=repetitionThreshold;
	}
}