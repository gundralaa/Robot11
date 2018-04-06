package Team4450.Robot11;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import Team4450.Lib.NavX;
import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMTalonSRX;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Devices
{
	// Motor CAN ID/PWM port assignments (1=left-front, 2=left-rear, 3=right-front, 4=right-rear)
	private final static WPI_TalonSRX	LFCanTalon = new WPI_TalonSRX(1), LRCanTalon = new WPI_TalonSRX(2), RFCanTalon = new WPI_TalonSRX(3), RRCanTalon = new WPI_TalonSRX(4);

	public static DifferentialDrive	robotDrive;

	public static WPI_TalonSRX        grabberMotorLeft = new WPI_TalonSRX(5), grabberMotorRight = new WPI_TalonSRX(6);
	public static PWMTalonSRX			winchMotor = new PWMTalonSRX(0);
	public static SpeedControllerGroup grabberMotors;

	public final static Joystick      utilityStick = new Joystick(2);	
	public final static Joystick      leftStick = new Joystick(0);	
	public final static Joystick		rightStick = new Joystick(1);	
	public final static Joystick		launchPad = new Joystick(3);

	public final static Compressor	compressor = new Compressor(0);	// Compressor class represents the PCM. There are 2.

	public final static ValveDA		gearShifter = new ValveDA(0);
	public final static ValveDA		grabberGrabValve = new ValveDA(2);
	public final static ValveDA		grabberWristValve = new ValveDA(4);
	public final static ValveDA		winchBrakeValve = new ValveDA(6);

	public final static AnalogInput	pressureSensor = new AnalogInput(0);

	public final static PowerDistributionPanel	PDP = new PowerDistributionPanel();

	public final static DriverStation				ds = DriverStation.getInstance();

	public static NavX				navx;

	public final static DigitalInput		winchLimitSwitch = new DigitalInput(6);

	public final static Servo				barReleaseServo = new Servo(2);
	public final static Servo				braceReleaseServo = new Servo(3);

	// Wheel encoder is plugged into dio port 0/2 - orange=+5v blue=signal, dio port 1/3 black=gnd yellow=signal. 
	public final static Encoder		driveEncoder1 = new Encoder(0, 1, true, EncodingType.k4X);
	public final static Encoder		driveEncoder2 = new Encoder(2, 3, true, EncodingType.k4X);

	// Winch encoder is plugged into dio port 4 - orange=+5v blue=signal, dio port 5 black=gnd yellow=signal. 
	public final static Encoder		winchEncoder = new Encoder(4, 5, true, EncodingType.k4X);

	// Encoder ribbon cable to dio ports: ribbon wire 2 = orange, 5 = yellow, 7 = blue, 10 = black
	// not used.

	// Create DifferentialDrive object for CAN Talon controllers.

	public static void InitializeCANTalonDrive()
	{
		Util.consoleLog();

		// Initialize CAN Talons and write status to log so we can verify
		// all the Talons are connected.
		InitializeCANTalon(LFCanTalon);
		InitializeCANTalon(LRCanTalon);
		InitializeCANTalon(RFCanTalon);
		InitializeCANTalon(RRCanTalon);

		InitializeCANTalon(grabberMotorLeft);
		grabberMotorLeft.setNeutralMode(NeutralMode.Brake);
		InitializeCANTalon(grabberMotorRight);
		grabberMotorRight.setNeutralMode(NeutralMode.Brake);

		// Configure CAN Talons with correct inversions.
		LFCanTalon.setInverted(false);
		LRCanTalon.setInverted(false);

		RFCanTalon.setInverted(false);
		RRCanTalon.setInverted(false);

		winchMotor.setInverted(Robot.isClone);

		grabberMotors = new SpeedControllerGroup(grabberMotorLeft, grabberMotorRight);

		// Turn on brake mode for CAN Talons.
		SetCANTalonBrakeMode(true);

		// Setup the SpeedControllerGroups for the left and right set of motors.
		SpeedControllerGroup LeftGroup = new SpeedControllerGroup(LFCanTalon, LRCanTalon);
		SpeedControllerGroup RightGroup = new SpeedControllerGroup(RFCanTalon, RRCanTalon);

		robotDrive = new DifferentialDrive(LeftGroup, RightGroup);
	}

	public static void resetServo() {
		Util.consoleLog();
		barReleaseServo.setPosition(0.5);
	}

	// Initialize and Log status indication from CANTalon. If we see an exception
	// or a talon has low voltage value, it did not get recognized by the RR on start up.

	public static void InitializeCANTalon(WPI_TalonSRX talon)
	{
		Util.consoleLog("talon init: %s   voltage=%.1f", talon.getDescription(), talon.getBusVoltage());

		talon.clearStickyFaults(0); //0ms means no blocking.
		//talon.enableControl();
		//talon.changeControlMode(ControlMode.PercentOutput); //TODO Find PercentVbus
	}

	// Set neutral behavior of CAN Talons. True = brake mode, false = coast mode.

	public static void SetCANTalonBrakeMode(boolean brakeMode)
	{
		Util.consoleLog("brakes on=%b", brakeMode);

		NeutralMode newMode;
		if (brakeMode) {
			newMode = NeutralMode.Brake;
		} else {
			newMode = NeutralMode.Coast;
		}

		LFCanTalon.setNeutralMode(newMode);
		LRCanTalon.setNeutralMode(newMode);
		RFCanTalon.setNeutralMode(newMode);
		RRCanTalon.setNeutralMode(newMode);
	}

	// Set CAN Talon voltage ramp rate. Rate is seconds.

	public static void SetCANTalonRampRate(double seconds)
	{
		Util.consoleLog("%f", seconds);

		LFCanTalon.configOpenloopRamp(seconds,0);
		LRCanTalon.configOpenloopRamp(seconds,0);
		RFCanTalon.configOpenloopRamp(seconds,0);
		RRCanTalon.configOpenloopRamp(seconds,0);
	}


	// Return voltage and current draw for each CAN Talon.

	public static String GetCANTalonStatus()
	{
		return String.format("%.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f", 
				LFCanTalon.getMotorOutputVoltage(), LFCanTalon.getOutputCurrent(),
				LRCanTalon.getMotorOutputVoltage(), LRCanTalon.getOutputCurrent(),
				RFCanTalon.getMotorOutputVoltage(), RFCanTalon.getOutputCurrent(),
				RRCanTalon.getMotorOutputVoltage(), RRCanTalon.getOutputCurrent());
	}
	
	private static boolean highGear = false;
	public static boolean isHighGear() { return highGear; }
	
	public static void toggleGear() {
		if (highGear)
			lowGear();
		else
			highGear();
	}
	
	public static void lowGear() {
		Util.consoleLog("Low Gear");
		Devices.gearShifter.SetA();
		SmartDashboard.putBoolean("Low", true);
		SmartDashboard.putBoolean("High", false);
		highGear = false;
	}
	
	public static void highGear() { //TODO Check Clone v Comp
		Util.consoleLog("High Gear");
		Devices.gearShifter.SetB();
		SmartDashboard.putBoolean("Low", false);
		SmartDashboard.putBoolean("High", true);
		highGear = true;
	}

	public static void updateNetworkTables() {
		SmartDashboard.putData("Sean's Debug Table/ControlSystem/Motors/Drive/LF", LFCanTalon);
		SmartDashboard.putData("Sean's Debug Table/ControlSystem/Motors/Drive/LR", LRCanTalon);
		SmartDashboard.putData("Sean's Debug Table/ControlSystem/Motors/Drive/RF", RFCanTalon);
		SmartDashboard.putData("Sean's Debug Table/ControlSystem/Motors/Drive/RR", RRCanTalon);

		SmartDashboard.putData("Sean's Debug Table/ControlSystem/Motors/Grabbers", grabberMotors);

		SmartDashboard.putData("Sean's Debug Table/ControlSystem/Motors/DriveGroup", robotDrive);

		if (navx != null) SmartDashboard.putData("Sean's Debug Table/ControlSystem/NavX", navx.getAHRS());

		if (PDP != null) SmartDashboard.putData("Sean's Debug Table/ControlSystem/PDP", PDP);
		if (compressor != null) SmartDashboard.putData("Sean's Debug Table/ControlSystem/Compressor", compressor);
		
		SmartDashboard.putData("Sean's Debug Table/Encoders/Drive1", driveEncoder1);
   		SmartDashboard.putData("Sean's Debug Table/Encoders/Drive2", driveEncoder1);
   		SmartDashboard.putData("Sean's Debug Table/Encoders/Winch", winchEncoder);
   		
   		SmartDashboard.putData("Sean's Debug Table/Servo/ForkDeploy", barReleaseServo);
   		SmartDashboard.putData("Sean's Debug Table/Servo/BraceDeploy", braceReleaseServo);
   		
   		if (Lift.getInstance() != null) SmartDashboard.putBoolean("Sean's Debug Table/PID/LiftEnabled", Lift.getInstance().isAutoLifting());
	}

}
