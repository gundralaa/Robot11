package Team4450.Robot11;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Timer;

public class Lift {

	private static Lift instance;
	public static Lift getInstance(Robot robot) {
		if (instance == null) instance = new Lift(robot);
		return instance;
	}

	//---------------------------------------------------

	private Robot robot;
	private PIDController pidController;
	private Lift(Robot robot) {
		this.robot = robot;
		pidController = new PIDController(0.1, 0.01, 0, Devices.encoder, Devices.liftMotor); //TODO Set correct encoder
	}

	public void extendWrist() {
		Util.consoleLog();
	}

	public void retractWrist() {
		Util.consoleLog();
	}

	private boolean clawOpen = false;
	public boolean getClawOpen() {return clawOpen; }
	
	public void openClaw() {
		Util.consoleLog();
		clawOpen = true;
	}

	public void closeClaw() {
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

	public void changeWinch() {
		Util.consoleLog();
	}


	public void ejectCube() {
		EjectThread ejectThread = new EjectThread(robot);
		ejectThread.start();
	}

	private IntakeThread intakeThread;
	public void intakeCube() {
		if (intakeThread == null) {
			intakeThread = new IntakeThread(robot);
			intakeThread.start();
		} else {
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
			Devices.grabberMotors.set(-1);
			Timer.delay(0.5);
			Lift.getInstance(robot).openClaw();
			Devices.grabberMotors.set(0);
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
			Lift.getInstance(robot).openClaw();
			Devices.grabberMotors.set(1);
			while (Devices.grabberMotor1.getOutputCurrent() < 12 && !isInterrupted())Timer.delay(0.02);
			Lift.getInstance(robot).closeClaw();
			Devices.grabberMotors.set(0);
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
		while (Math.abs(pid.getSetpoint()-Devices.encoder.get()) > 50 && pid.get() > 0.3 && !isInterrupted()) { Timer.delay(0.2); }
		pid.disable();
	}
}
