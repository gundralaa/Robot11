package Team4450.Robot11;

import edu.wpi.first.wpilibj.SpeedController;

public class WinchMotor implements SpeedController {

	public static WinchMotor getInstance() {
		if (instance == null) {
			instance = new WinchMotor();
		}
		return instance;
	}
	private static WinchMotor instance;
	
	private WinchMotor() {
	}
	
	//--------------------------------------------------------------
	
	private boolean inverted = false;
	@Override
	public void pidWrite(double output) {
		set(output);
	}

	@Override
	public void set(double speed) {
		set(speed, false);
	}
	
	public void set(double speed, boolean extendClimbLift) {
		//TODO Insert magic.
	}

	@Override
	public double get() {
		return Devices.winchMotor1.get();
	}

	@Override
	public void setInverted(boolean isInverted) {
		inverted = isInverted;
	}

	@Override
	public boolean getInverted() {
		return inverted;
	}

	@Override
	public void disable() {
		Devices.winchMotor1.disable();
		Devices.winchMotor2.disable();
	}

	@Override
	public void stopMotor() {
		Devices.winchMotor1.stopMotor();
		Devices.winchMotor2.stopMotor();
	}
	
	public void dispose() {
		instance = null;
	}

}
