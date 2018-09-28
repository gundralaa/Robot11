package Team4450.Robot11;

import Team4450.Lib.*;

public class GearTrain {
	
	static boolean lowSpeed = true;
	static boolean highSpeed = false;
	
	public static void setLow() {
		Devices.gearShiftValve.SetA();
		Devices.SetCANTalonBrakeMode(false);
		highSpeed = false;
		lowSpeed = true;
		
	}
	
	public static void setHigh() {
		Devices.gearShiftValve.SetB();
		Devices.SetCANTalonBrakeMode(true);
		highSpeed = true;
		lowSpeed = false;
	}
	
	public static boolean isLowSpeed() {
		return lowSpeed;
	}
	
	public static boolean isHighSpeed() {
		return highSpeed;
	}

}
