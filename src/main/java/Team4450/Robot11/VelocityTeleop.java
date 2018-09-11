package Team4450.Robot11;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import Team4450.Lib.Util;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class VelocityTeleop 
{
	private final Robot	robot;
	
	/** Hardware */
	TalonSRX LRCanTalon = new TalonSRX(2);
	TalonSRX RRCanTalon = new TalonSRX(1);
	Joystick _gamepad = new Joystick(0);
	
	/** Latched values to detect on-press events for buttons and POV */
	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];
	
	/** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;

	VelocityTeleop(Robot robot)
	{
		Util.consoleLog();
		
		// Motor safety turned off during initialization.
		Devices.robotDrive.setSafetyEnabled(false);

		this.robot = robot;
	}
	
	public void OperatorControl() throws Exception
	{
		Util.consoleLog();
		
		/* Disable all motors */
		RRCanTalon.set(ControlMode.PercentOutput, 0);
		LRCanTalon.set(ControlMode.PercentOutput,  0);
		
		/* Set neutral modes */
		LRCanTalon.setNeutralMode(NeutralMode.Brake);
		RRCanTalon.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		
		/* Configure the left Talon's selected sensor as local QuadEncoder */
		LRCanTalon.configSelectedFeedbackSensor(	FeedbackDevice.CTRE_MagEncoder_Relative,// Local Feedback Source
													Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
													Constants.kTimeoutMs);					// Configuration Timeout

		/* Configure the left Talon's selected sensor as a remote sensor for the right Talon */
		RRCanTalon.configRemoteFeedbackFilter(LRCanTalon.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
												Constants.REMOTE_0,							// Source number [0, 1]
												Constants.kTimeoutMs);						// Configuration Timeout
		
		/* Setup Sum signal to be used for Distance */
		RRCanTalon.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);				// Feedback Device of Remote Talon
		RRCanTalon.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs);	// Quadrature Encoder of current Talon
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		RRCanTalon.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													Constants.PID_PRIMARY,
													Constants.kTimeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		RRCanTalon.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														Constants.PID_PRIMARY,		// PID Slot of Source 
														Constants.kTimeoutMs);		// Configuration Timeout
		
		/* Configure output and sensor direction */
		LRCanTalon.setInverted(false);
		LRCanTalon.setSensorPhase(true);
		RRCanTalon.setInverted(true);
		RRCanTalon.setSensorPhase(true);
		
		/* Set status frame periods to ensure we don't have stale data */
		RRCanTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		RRCanTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		LRCanTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);

		/* Configure neutral deadband */
		RRCanTalon.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		LRCanTalon.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);

		/* Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		LRCanTalon.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		LRCanTalon.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		RRCanTalon.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		RRCanTalon.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		
		/* FPID Gains for velocity servo */
		RRCanTalon.config_kP(Constants.kSlot_Velocit, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		RRCanTalon.config_kI(Constants.kSlot_Velocit, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		RRCanTalon.config_kD(Constants.kSlot_Velocit, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
		RRCanTalon.config_kF(Constants.kSlot_Velocit, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		RRCanTalon.config_IntegralZone(Constants.kSlot_Velocit, (int)Constants.kGains_Velocit.kIzone, Constants.kTimeoutMs);
		RRCanTalon.configClosedLoopPeakOutput(Constants.kSlot_Velocit, Constants.kGains_Velocit.kPeakOutput, Constants.kTimeoutMs);
		RRCanTalon.configAllowableClosedloopError(Constants.kSlot_Velocit, 0, Constants.kTimeoutMs);
			
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		RRCanTalon.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 0, Constants.kTimeoutMs);
		RRCanTalon.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 1, Constants.kTimeoutMs);

		/* configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		RRCanTalon.configAuxPIDPolarity(false, Constants.kTimeoutMs);

		/* Initialize */
		_firstCall = true;
		_state = false;
		zeroSensors();

		Util.consoleLog("enter driving loop");
		
		Devices.robotDrive.setSafetyEnabled(true);
		
		while (robot.isEnabled() && robot.isOperatorControl())
		{
			/* JS processing */
			double forward = -1 * Devices.rightStick.getY();
			double turn = Devices.rightStick.getX();
			forward = Deadband(forward);
			turn = Deadband(turn);
		
			/* Button processing for state toggle and sensor zeroing */
			getButtons(btns, _gamepad);
			
			if(btns[2] && !_btns[2])
			{
				_state = !_state; 	// Toggle state
				_firstCall = true;	// State change, do first call operation
			}
			else if (btns[1] && !_btns[1]) 
			{
				zeroSensors();		// Zero sensors
			}
			
			System.arraycopy(btns, 0, _btns, 0, Constants.kNumButtonsPlusOne);
			
			if(!_state)
			{
				if (_firstCall)
					System.out.println("This is a basic arcade drive.\n");
				
				LRCanTalon.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
				RRCanTalon.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
			}
			else
			{
				if (_firstCall) 
				{
					System.out.println("This is Velocity Closed Loop with a custom Feed Forward.");
					System.out.println("Travel [-500, 500] RPM while having the ability to add a FeedForward with joyX ");
					zeroSensors();
					
					/* Determine which slot affects which PID */
					RRCanTalon.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
				}
				
				/* Calculate targets from JS inputs, 500 rpm max */
				double target_RPM = forward * 500; /* +- 500 RPM */
				double target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0;
				double feedFwdTerm = turn * 0.25; /* how much to add to the close loop output */
				
				/* Configured for Velocity Closed Loop on Quad Encoders' Sum and Arbitrary FeedForward on joyX */
				RRCanTalon.set(ControlMode.Velocity, target_unitsPer100ms, DemandType.ArbitraryFeedForward, feedFwdTerm);
				
				LRCanTalon.follow(RRCanTalon);
			}
			
			/* Recreated variables */
			_firstCall = false;
			
			Timer.delay(.020);	// wait 20ms for update from driver station.
		}
	}
	
	/* Zeroes Quad Encoders on Talons */
	void zeroSensors() 
	{
		Devices.leftEncoder.reset();
		Devices.rightEncoder.reset();
		Devices.leftEncoder.resetMaxRate();
		Devices.rightEncoder.resetMaxRate();

		Util.consoleLog("[SRXEncoders] All sensors are zeroed.");
	}
	
	/** Deadband 5 percent, used on the gamepad */
	double Deadband(double value) 
	{
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
	}
	
	/** Gets all buttons from gamepad */
	void getButtons(boolean[] btns, Joystick gamepad) 
	{
		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) 
		{
			btns[i] = gamepad.getRawButton(i);
		}
	}
}
