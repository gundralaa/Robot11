package Team4450.Robot11;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import Team4450.Lib.NavX;
import Team4450.Lib.Util;
import Team4450.Lib.ValveDA;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Ultrasonic;

public class Devices
{
	  // Motor CAN ID/PWM port assignments (1=left-front, 2=left-rear, 3=right-front, 4=right-rear)
	  private static CANTalon	LFCanTalon, LRCanTalon, RFCanTalon, RRCanTalon, LSlaveCanTalon, RSlaveCanTalon;
	  
	  public static RobotDrive	robotDrive;

	  public final static Joystick      utilityStick = new Joystick(2);	
	  public final static Joystick      leftStick = new Joystick(0);	
	  public final static Joystick		rightStick = new Joystick(1);	
	  public final static Joystick		launchPad = new Joystick(3);

	  public final static Compressor	compressor = new Compressor(0);	// Compressor class represents the PCM. There are 2.
	  
	  public final static AnalogInput	pressureSensor = new AnalogInput(0);
	  
	  public final static PowerDistributionPanel	PDP = new PowerDistributionPanel();

	  public final static DriverStation				ds = DriverStation.getInstance();

	  public static NavX				navx;

	  // Create RobotDrive object for CAN Talon controllers.
	  
	  public static void InitializeCANTalonDrive()
	  {
		  Util.consoleLog();

		  LFCanTalon = new CANTalon(1);
		  LRCanTalon = new CANTalon(2);
		  RFCanTalon = new CANTalon(3);
		  RRCanTalon = new CANTalon(4);
		  LSlaveCanTalon = new CANTalon(5);
		  RSlaveCanTalon = new CANTalon(6);
		  
		  robotDrive = new RobotDrive(LFCanTalon, LRCanTalon, RFCanTalon, RRCanTalon);

	      // Initialize CAN Talons and write status to log so we can verify
	      // all the talons are connected.
	      InitializeCANTalon(LFCanTalon);
	      InitializeCANTalon(LRCanTalon);
	      InitializeCANTalon(RFCanTalon);
	      InitializeCANTalon(RRCanTalon);
	      InitializeCANTalon(LSlaveCanTalon);
	      InitializeCANTalon(RSlaveCanTalon);
	      
	      // Configure slave CAN Talons to follow the front L & R Talons.
	      LSlaveCanTalon.changeControlMode(TalonControlMode.Follower);
	      LSlaveCanTalon.set(LFCanTalon.getDeviceID());
	      LSlaveCanTalon.reverseOutput(true);

	      RSlaveCanTalon.changeControlMode(TalonControlMode.Follower);
	      RSlaveCanTalon.set(RFCanTalon.getDeviceID());
	      RSlaveCanTalon.reverseOutput(true);
	      
	      // Turn on brake mode for CAN Talons.
	      SetCANTalonBrakeMode(true);
	  }

	  // Initialize and Log status indication from CANTalon. If we see an exception
	  // or a talon has low voltage value, it did not get recognized by the RR on start up.
	  
	  public static void InitializeCANTalon(CANTalon talon)
	  {
		  Util.consoleLog("talon init: %s   voltage=%.1f", talon.getDescription(), talon.getBusVoltage());

		  talon.clearStickyFaults();
		  talon.enableControl();
		  talon.changeControlMode(TalonControlMode.PercentVbus);
	  }
	  
	  // Set neutral behavior of CAN Talons. True = brake mode, false = coast mode.

	  public static void SetCANTalonBrakeMode(boolean brakeMode)
	  {
		  Util.consoleLog("brakes on=%b", brakeMode);
		  
		  LFCanTalon.enableBrakeMode(brakeMode);
		  LRCanTalon.enableBrakeMode(brakeMode);
		  RFCanTalon.enableBrakeMode(brakeMode);
		  RRCanTalon.enableBrakeMode(brakeMode);
		  LSlaveCanTalon.enableBrakeMode(brakeMode);
		  RSlaveCanTalon.enableBrakeMode(brakeMode);
	  }
	  
	  // Set CAN Talon voltage ramp rate. Rate is volts/sec and can be 2-12v.
	  
	  public static void SetCANTalonRampRate(double rate)
	  {
		  Util.consoleLog("%f", rate);
		  
		  LFCanTalon.setVoltageRampRate(rate);
		  LRCanTalon.setVoltageRampRate(rate);
		  RFCanTalon.setVoltageRampRate(rate);
		  RRCanTalon.setVoltageRampRate(rate);
		  LSlaveCanTalon.setVoltageRampRate(rate);
		  RSlaveCanTalon.setVoltageRampRate(rate);
	  }
	  
	  // Return voltage and current draw for each CAN Talon.
	  
	  public static String GetCANTalonStatus()
	  {
		  return String.format("%.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f", 
				  LFCanTalon.getOutputVoltage(), LFCanTalon.getOutputCurrent(),
				  LRCanTalon.getOutputVoltage(), LRCanTalon.getOutputCurrent(),
				  RFCanTalon.getOutputVoltage(), RFCanTalon.getOutputCurrent(),
				  RRCanTalon.getOutputVoltage(), RRCanTalon.getOutputCurrent(),
				  LSlaveCanTalon.getOutputVoltage(), LSlaveCanTalon.getOutputCurrent(),
				  RSlaveCanTalon.getOutputVoltage(), RSlaveCanTalon.getOutputCurrent());
	  }

}
