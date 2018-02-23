package Team4450.Robot11;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift {

	private static Lift instance;
	public static Lift getInstance(Robot robot) {
		if (instance == null) instance = new Lift(robot);
		return instance;
	}

	//-----------------------------------------------------------------------------------------------

	private Robot robot;
	private PIDController pidController;
	private Lift(Robot robot) {
		this.robot = robot;
		pidController = new PIDController(0.1, 0.01, 0, Devices.winchEncoder, Devices.winchMotor);
	}

	public void extendWrist() {
		Devices.grabberWristValve.SetB();
		SmartDashboard.putBoolean("Deployed", true);
		Util.consoleLog();
	}

	public void retractWrist() {
		Devices.grabberWristValve.SetA();
		SmartDashboard.putBoolean("Deployed", false);
		Util.consoleLog();
	}

	private boolean clawOpen = false;
	public boolean getClawOpen() {return clawOpen; }
	
	public void openClaw() {
		Devices.grabberGrabValve.SetA();
		SmartDashboard.putBoolean("Grabber", true);
		Util.consoleLog();
		clawOpen = true;
	}

	public void closeClaw() {
		Devices.grabberGrabValve.SetB();
		SmartDashboard.putBoolean("Grabber", false);
		Util.consoleLog();
		clawOpen = false;
	}
	
	public static enum LiftHeight {
		GROUND (0), SWITCH (100), SCALE (200); //FIXME Get correct encoder counts
		private int encoderCount;
		LiftHeight(int encoderCount) {
			this.encoderCount = encoderCount;
		}
		
		public int getEncoderCount() { return encoderCount; }
	};
		
	public void setLiftHeight(LiftHeight height) {
		pidController.enable();
		pidController.setSetpoint(height.getEncoderCount());
		PIDChecker pidChecker = new PIDChecker(pidController);
		pidChecker.start();
	}


	public void ejectCube() {
		SmartDashboard.putBoolean("Spit", true);
		SmartDashboard.putBoolean("Intake", false);
		EjectThread ejectThread = new EjectThread(robot);
		ejectThread.start();
	}

	private IntakeThread intakeThread;
	public void toggleIntakeCube() {
		SmartDashboard.putBoolean("Spit", false);
		if (intakeThread == null) {
			SmartDashboard.putBoolean("Intake", true);
			intakeThread = new IntakeThread(robot);
			intakeThread.start();
		} else {
			SmartDashboard.putBoolean("Intake", false);
			clearIntakeThread();
		}
	}
	
	public void clearIntakeThread() {
		if (intakeThread != null) try { intakeThread.interrupt(); } catch(Exception e) {}
		intakeThread = null;
	}
}

class EjectThread extends Thread {
	private Robot robot;
	EjectThread(Robot robot) {
		this.robot = robot;
	}

	public void run() {
		if (robot.isEnabled()) {
			Devices.grabberMotors.set(-1); //TODO Adjust eject speed
			Timer.delay(0.5); //TODO Adjust eject time delay
			Lift.getInstance(robot).openClaw();
			Devices.grabberMotors.set(0);
			SmartDashboard.putBoolean("Spit", false);
		}
	}
}

class IntakeThread extends Thread {
	private Robot robot;
	IntakeThread(Robot robot) {
		this.robot = robot;
	}

	public void run() {
		if (robot.isEnabled()) {
			SmartDashboard.putBoolean("AutoGrab", true);
			Lift.getInstance(robot).openClaw();
			Devices.grabberMotors.set(1);
			while (Devices.grabberMotorLeft.getOutputCurrent() < 12 && !isInterrupted() && robot.isEnabled()) { Timer.delay(0.02); } //TODO Determine correct output current value
			Lift.getInstance(robot).closeClaw();
			Devices.grabberMotors.set(0);
			SmartDashboard.putBoolean("AutoGrab", false);
			Lift.getInstance(robot).clearIntakeThread();
		}
	}
}

class PIDChecker extends Thread {
	private PIDController pid;
	PIDChecker(PIDController pid) {
		this.pid = pid;
	}
	
	public void run() {
		while (Math.abs(pid.getSetpoint()-Devices.winchEncoder.get()) > 50 && pid.get() > 0.3 && !isInterrupted()) { Timer.delay(0.2); } //TODO Determine correct tolerances.
		pid.disable();
	}
}
