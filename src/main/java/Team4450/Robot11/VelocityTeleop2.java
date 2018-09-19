package Team4450.Robot11;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import Team4450.Lib.LCD;
import Team4450.Lib.Util;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;

public class VelocityTeleop2 
{
	Robot	robot;
	
	/** Hardware */
	TalonSRX _leftMaster = Devices.LRCanTalon;
	TalonSRX _rightMaster = Devices.RRCanTalon;
	
	Joystick _gamepad = new Joystick(1);
	
	/** Latched values to detect on-press events for buttons */
	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];
	
	/** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	double _lockedDistance = 0;
	double _targetAngle = 0;
	
	VelocityTeleop2(Robot robot)
	{
		Util.consoleLog();
		
		// Motor safety turned off during initialization.
		Devices.robotDrive.setSafetyEnabled(false);

		this.robot = robot;
	}
	
	public void OperatorControl() 
	{
		int		count = 0, totalRpm = 0, avgRpm = 0, totalError = 0, avgError = 0;
		double 	_targetAngle = 0, target_unitsPer100ms = 0;
		
		final int MAX_RPM = 258;
		
		Util.consoleLog();		
		
		//LRCanTalon.setInverted(false);
		Devices.LRCanTalon.setSensorPhase(false);
		
		Devices.RRCanTalon.setInverted(false);
		Devices.RRCanTalon.setSensorPhase(false);
		Devices.RFCanTalon.setInverted(false);
		
		/* Disable all motor controllers */
		_rightMaster.set(ControlMode.PercentOutput, 0);
		_leftMaster.set(ControlMode.PercentOutput, 0);
		
		/* Set Neutral Mode */
		_leftMaster.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);
		
		/** Feedback Sensor Configuration */
		
		/* Configure the right Talon's selected sensor to a Quad Encoder*/
		_leftMaster.configSelectedFeedbackSensor(	FeedbackDevice.QuadEncoder, 			// Local Feedback Source
													Constants.PID_PRIMARY,					// PID Slot for Source [0, 1]
													Constants.kTimeoutMs);					// Configuration Timeout
	
		/* Configure the Remote (left) Talon's selected sensor as a remote sensor for the right Talon */
		_rightMaster.configRemoteFeedbackFilter(_leftMaster.getDeviceID(),					// Device ID of Source
												RemoteSensorSource.TalonSRX_SelectedSensor,	// Remote Feedback Source
												Constants.REMOTE_0,							// Source number [0, 1]
												Constants.kTimeoutMs);						// Configuration Timeout
		
		/* Setup Sum signal to be used for Distance */
		_rightMaster.configSensorTerm(SensorTerm.Sum0, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);				// Feedback Device of Remote Talon
		_rightMaster.configSensorTerm(SensorTerm.Sum1, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs);	// Quadrature Encoder of current Talon
		
		/* Setup Difference signal to be used for Turn */
		_rightMaster.configSensorTerm(SensorTerm.Diff1, FeedbackDevice.RemoteSensor0, Constants.kTimeoutMs);
		_rightMaster.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kTimeoutMs);
		
		/* Configure Sum [Sum of both QuadEncoders] to be used for Primary PID Index */
		_rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.SensorSum, 
													Constants.PID_PRIMARY,
													Constants.kTimeoutMs);
		
		/* Scale Feedback by 0.5 to half the sum of Distance */
		_rightMaster.configSelectedFeedbackCoefficient(	0.5, 						// Coefficient
														Constants.PID_PRIMARY,		// PID Slot of Source 
														Constants.kTimeoutMs);		// Configuration Timeout
		
		/* Configure Difference [Difference between both QuadEncoders] to be used for Auxiliary PID Index */
		_rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.SensorDifference, 
													Constants.PID_TURN, 
													Constants.kTimeoutMs);
		
		/* Scale the Feedback Sensor using a coefficient */
		_rightMaster.configSelectedFeedbackCoefficient(	Constants.kTurnTravelUnitsPerRotation / Constants.kEncoderUnitsPerRotation,
														Constants.PID_TURN, 
														Constants.kTimeoutMs);		
		
		Util.consoleLog("turn feedback coeff=%.3f", Constants.kTurnTravelUnitsPerRotation / Constants.kEncoderUnitsPerRotation);
		
		/* Set status frame periods to ensure we don't have stale data */
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 20, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 20, Constants.kTimeoutMs);		
		_leftMaster.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, Constants.kTimeoutMs);
	
		/* Configure neutral deadband */
		_rightMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		_leftMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
	
		/* Max out the peak output (for all modes).  
		 * However you can limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_leftMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		_rightMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_rightMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
	
		/* FPID Gains for velocity */
		_rightMaster.config_kP(Constants.kSlot_Velocit, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
		_rightMaster.config_kI(Constants.kSlot_Velocit, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
		_rightMaster.config_kD(Constants.kSlot_Velocit, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
		_rightMaster.config_kF(Constants.kSlot_Velocit, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
		_rightMaster.config_IntegralZone(Constants.kSlot_Velocit, (int)Constants.kGains_Velocit.kIzone, Constants.kTimeoutMs);
		_rightMaster.configClosedLoopPeakOutput(Constants.kSlot_Velocit, Constants.kGains_Velocit.kPeakOutput, Constants.kTimeoutMs);
		_rightMaster.configAllowableClosedloopError(Constants.kSlot_Velocit, 0, Constants.kTimeoutMs);
	
		/* FPID Gains for turn */
		_rightMaster.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
		_rightMaster.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
		_rightMaster.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
		_rightMaster.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
		_rightMaster.config_IntegralZone(Constants.kSlot_Turning, (int)Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);
		_rightMaster.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);
		_rightMaster.configAllowableClosedloopError(Constants.kSlot_Turning, 0, Constants.kTimeoutMs);
			
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_rightMaster.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 0, Constants.kTimeoutMs);
		_rightMaster.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 1, Constants.kTimeoutMs);
	
		/* configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		_rightMaster.configAuxPIDPolarity(false, Constants.kTimeoutMs);
	
		/* Initialize */
		_firstCall = true;
		_state = false;
		zeroSensors();

		Util.consoleLog("enter driving loop");
		
		//Devices.robotDrive.setSafetyEnabled(true);
		
		while (robot.isEnabled() && robot.isOperatorControl())
		{
			/* Gamepad processing */
			double forward = -1 * Devices.rightStick.getY();
			double turn = Devices.rightStick.getX() * -1;
			forward = Deadband(forward);
			turn = Deadband(turn);
		
			/* Button processing for state toggle and sensor zeroing */
			getButtons(btns, Devices.rightStick);
			
			if(btns[2] && !_btns[2])
			{
				_state = !_state; 	// Toggle state
				_firstCall = true;	// State change, do first call operation
				// The effect of the next statement is to set the rotation pid (aux) target (setpoint) to
				// the current difference between the encoder counts, so when the aux pid loop
				// runs, it will maintain the current robot heading.
				_targetAngle =  _rightMaster.getSelectedSensorPosition(Constants.PID_TURN);
				
				Util.consoleLog("mode shift, targetAngle=%d", _rightMaster.getSelectedSensorPosition(Constants.PID_TURN));
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
				
				_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
				_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
			}
			else
			{
				if (_firstCall) 
				{
					System.out.println("This is Drive Straight Velocity with the Auxiliary feature using encoder difference.");
					System.out.println("Travel in either direction while also maintaining a straight heading.\n");
					
					/* Determine which slot affects which PID */
					_rightMaster.selectProfileSlot(Constants.kSlot_Velocit, Constants.PID_PRIMARY);
					_rightMaster.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
				}	
				
				/* Calculate targets from gamepad inputs */
				double target_RPM = forward * MAX_RPM; 
				target_unitsPer100ms = target_RPM * Constants.kSensorUnitsPerRotation / 600.0;
				_targetAngle += turn * 100;
				
				/* So the velocity pid control on the primary pid loop controls the speed trying to 
				 * vary motor output to keep speed at the target set below. The steering is done by
				 * the second, or auxiliary pid loop. Here the loop tries to adjust motor output to
				 * to keep the difference between the left and right encoder counts to match the
				 * value in _targetAngle. Starting from zero, the aux pid loop will try to keep the
				 * encoder count difference at zero, driving in a straight line. So the code is designed
				 * to allow you to run in mode 1, power level control and rotate the robot around.
				 * This rotation is in effect tracked as the accumulated difference in the encoder
				 * counts. When you switch to mode 2, velocity and turn control via pid  loops,
				 * you start the _targetAngle with the starting difference, which represents the
				 * heading of the robot at switch time. You run from there with the pid loop 
				 * maintaining the difference which makes the robot start out after switch with the
				 * same heading it had before the switch.
				 * 
				 * To then turn (change heading), we change the _targetAngle by adding the joystick Y
				 * value creating a new heading. Its tricky because we say angle and heading, but the
				 * code is 
				 */
				
				/* Configured for Velocity Closed Loop on Quad Encoders' Sum and Auxiliary PID on Quad Encoders' Difference */
				
				_rightMaster.set(ControlMode.Velocity, target_unitsPer100ms, DemandType.AuxPID, _targetAngle);
				
				// Both pid loops are run on the right talon. It is configured to read the left talon
				// encoder and so use both in the pid loops. Then we configure the left talon to follow
				// the right. The talons are configured above to work in concert based on the pid loop
				// outputs on the right talon.
				
				_leftMaster.follow(_rightMaster, FollowerType.AuxOutput1);
			}
			
			_firstCall = false;
		
			count++;
			totalRpm += Devices.rightEncoder.getRPM();
			avgRpm = totalRpm / count;
			
			totalError += Math.abs(_rightMaster.getClosedLoopError(Constants.PID_PRIMARY));
			avgError = totalError  / count;
			
			if (count > 100) totalRpm = count = totalError = 0;

			Util.consoleLog("ry=%.2f rx=%.2f ta=%.2f mo=%.2f  vel=%d  rpm=%d", forward, turn, _targetAngle, 
					_rightMaster.getMotorOutputPercent(),
					_rightMaster.getSelectedSensorVelocity(0), Devices.rightEncoder.getRPM());
			
			LCD.printLine(4, "ry=%.2f rx=%.2f  mo=%.2f  vel=%d  rpm=%d", forward, turn,
					_rightMaster.getMotorOutputPercent(),
					_rightMaster.getSelectedSensorVelocity(0), Devices.rightEncoder.getRPM());
			
			LCD.printLine(6, "err=%d  targetVel=%.2f  turn(y)=%.2f  ta=%.2f", 
					Devices.RRCanTalon.getClosedLoopError(Constants.PID_PRIMARY),
					target_unitsPer100ms, turn, _targetAngle);
			
			LCD.printLine(8, "lmo=%.2f  LVel=%d", _rightMaster.getMotorOutputPercent(),
					_rightMaster.getSelectedSensorVelocity(0));			
			
			LCD.printLine(9, "avgerr=%d", avgError);
			
			LCD.printLine(10, "rpm=%d avg=%d  max=%d  maxrate=%d", Devices.rightEncoder.getRPM(),
					avgRpm, Devices.rightEncoder.getMaxRPM(), 
					Devices.rightEncoder.getMaxRate(PIDRateType.ticksPer100ms));
			
			Timer.delay(.020);	// wait 20ms for update from driver station.			
		}
	}
	
	/** Zeroes Quad Encoders on Talons */
	void zeroSensors() 
	{
		_leftMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		_rightMaster.getSensorCollection().setQuadraturePosition(0, Constants.kTimeoutMs);
		System.out.println("[QuadEncoder] All sensors are zeroed.\n");
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
		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
			btns[i] = gamepad.getRawButton(i);
		}
	}
}
