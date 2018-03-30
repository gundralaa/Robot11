package Team4450.Robot11;

import Team4450.Lib.LCD;
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
	
	public static Lift getInstance() {
		if (instance != null) return instance;
		else return null;
	}

	//-----------------------------------------------------------------------------------------------

	private Robot robot;
	private PIDController pidController;
	private boolean toggleOverride = false;
	private Lift(Robot robot) {
		this.robot = robot;
		pidController = new PIDController(0.0003, 0.00001, 0.0003, 0.0, Devices.winchEncoder, Devices.winchMotor);
		pidController.setOutputRange(-1, 1);
		pidController.setPercentTolerance(1); //1%
		
		SmartDashboard.putData("Sean's Debug Table/Lift/PID", pidController);
	}
	
	public PIDController getPID() {
		return pidController;
	}

	public void extendWrist() {
		if (Robot.isComp) Devices.grabberWristValve.SetA();
		else Devices.grabberWristValve.SetB();
		SmartDashboard.putBoolean("Deployed", true);
		Util.consoleLog();
	}

	public void retractWrist() {
		if (Robot.isComp) Devices.grabberWristValve.SetB();
		else Devices.grabberWristValve.SetA();
		SmartDashboard.putBoolean("Deployed", false);
		Util.consoleLog();
	}

	private boolean clawOpen = false;
	public boolean getClawOpen() {return clawOpen; }
	
	public void openClaw() {
		if (Robot.isComp) Devices.grabberGrabValve.SetB();
		else Devices.grabberGrabValve.SetA();
		SmartDashboard.putBoolean("Grabber", true);
		Util.consoleLog();
		clawOpen = true;
	}

	public void closeClaw() {
		if (Robot.isComp) Devices.grabberGrabValve.SetA();
		else Devices.grabberGrabValve.SetB();
		SmartDashboard.putBoolean("Grabber", false);
		Util.consoleLog();
		clawOpen = false;
	}
	
	public static enum LiftHeight {
		GROUND (0), EXCHANGE (900), SWITCH (7900, 10100), SCALE (0) /* Scale Currently Unused */, CLIMB (12100, 13300); //TODO Get correct encoder counts for exchange and scale
		private int encoderCountComp;
		private int encoderCountClone;
		
		LiftHeight(int encoderCount) {
			this.encoderCountComp = encoderCount;
			this.encoderCountClone = encoderCount;
		}
		
		LiftHeight(int encoderCountComp, int encoderCountClone) {
			this.encoderCountComp = encoderCountComp;
			this.encoderCountClone = encoderCountClone;
		}
		
		public int getEncoderCount() { return (Robot.isComp ? encoderCountComp : encoderCountClone); }
	};
		
	public void setLiftHeight(LiftHeight height) {
		Util.consoleLog();
		pidController.enable();
		pidController.setSetpoint(height.getEncoderCount());
		PIDChecker pidChecker = new PIDChecker(pidController, robot);
		pidChecker.start();
	}
	
	public boolean isAutoLifting() {
		return pidController.isEnabled();
	}
	
	public void stopAutoLift() {
		pidController.disable();
	}

	public void toggleOverride() {
		toggleOverride = !toggleOverride;
		SmartDashboard.putBoolean("Override", toggleOverride);
	}
	
	public void releaseBrace() {
		if (Devices.winchEncoder.get() > 6500) Devices.braceReleaseServo.setAngle(60);
	}
	
	public enum ForkReleaseState { RETRACT(0.2), HALF(0.5), RELEASE(0.82);
		double position;
		ForkReleaseState(double position) {
			this.position = position;
		}
		
		public double getPosition() {
			return position;
		}
	};
	
	public void setForkRelease(ForkReleaseState newForkState) {
		Devices.forkReleaseServo.setPosition(newForkState.getPosition());
	}
	
	public void setMotor(double power) {
		if (isAutoLifting() && power != 0) {
			pidController.disable();
		} else if (isAutoLifting()) {
			return;
		}
		if (toggleOverride || power == 0) {
			Devices.winchMotor.set(power);
		} else if ((Devices.winchEncoder.get() >= (Robot.isComp ? 13600 : 14000) && power > 0) || ((Robot.isClone ? !Devices.winchLimitSwitch.get() : Devices.winchLimitSwitch.get()) && power < 0))  {
			Devices.winchMotor.set(0);
		} else {
			Devices.winchMotor.set(power);
		}
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
			Devices.grabberMotors.set(-.5);
			Timer.delay(0.5); //TODO Adjust eject time delay
			Lift.getInstance(robot).openClaw();
			Devices.grabberMotors.set(0);
			SmartDashboard.putBoolean("Spit", false);
		}
	}
}

class IntakeThread extends Thread {
	private Robot robot;
	private double stopCurrent;
	IntakeThread(Robot robot) {
		this.robot = robot;
		stopCurrent = (Robot.isClone ? 18.0 : 15.0);
	}

	public void run() {
		if (robot.isEnabled()) {
			Util.consoleLog("AutoIntake Start");
			SmartDashboard.putBoolean("AutoGrab", true);
			Lift.getInstance(robot).openClaw();
			Devices.grabberMotors.set(.5);
			while (Devices.grabberMotorLeft.getOutputCurrent() < stopCurrent && !isInterrupted() && robot.isEnabled()) { LCD.printLine(10, "Intake Voltage=%f", Devices.grabberMotorLeft.getOutputCurrent());Timer.delay(0.02); }
			Devices.grabberMotors.set(0);
			SmartDashboard.putBoolean("AutoGrab", false);
			Util.consoleLog("AutoIntake End");
			Lift.getInstance(robot).clearIntakeThread();
		}
	}
}

class PIDChecker extends Thread {
	private PIDController pid;
	private Robot robot;
	PIDChecker(PIDController pid, Robot robot) {
		this.pid = pid;
		this.robot = robot;
	}
	
	public void run() {
		//while (Math.abs(pid.getSetpoint()-Devices.winchEncoder.get()) > 50 && pid.get() > 0.2 && !isInterrupted() && robot.isEnabled() && pid.isEnabled()) { Timer.delay(0.2); } //TODO Determine correct tolerances.
		while (!isInterrupted() && robot.isEnabled() && pid.isEnabled()) { Timer.delay(0.2); }
		pid.disable();
		Util.consoleLog();
	}
}
