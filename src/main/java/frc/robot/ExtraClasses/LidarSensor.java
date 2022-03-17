package frc.robot.ExtraClasses;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.filter.MedianFilter;
import static frc.robot.Constants.*;


public class LidarSensor {

	// Offset for cm
	private static final int CALIBRATION_OFFSET = 0;
	private MedianFilter lidarFilter;

	private Counter counter;

	// Creates a new LidarSensor object, takes in the DIO port the sensor is
	// connected to and configures pulses
	public LidarSensor(DigitalSource source) {

		counter = new Counter(source);
		counter.setMaxPeriod(1.0);
		counter.setSemiPeriodMode(true);
		counter.reset();

		lidarFilter = new MedianFilter(150);

	}

	// Returns the distance of the sensor in inches
	public double getDistance() {

		double cm;
		double inches;
		cm = (lidarFilter.calculate(counter.getPeriod()) * 1000000.0 / 10.0) + CALIBRATION_OFFSET;
		inches = (cm / 2.54);
		SmartDashboard.putNumber("Lidar distance (inches)", inches);
		return inches;
	}

	public void displayInches() {
		double cm = (lidarFilter.calculate(counter.getPeriod()) * 1000000.0 / 10.0) + CALIBRATION_OFFSET;
		double inches = (cm / 2.54);
		SmartDashboard.putNumber("inches", inches);
	}
}

