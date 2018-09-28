package Team4450.Robot11;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;

import Team4450.Lib.NavX;
import Team4450.Lib.SRXMagneticEncoderRelative.PIDRateType;
import Team4450.Lib.Util;
import Team4450.Lib.*;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;

public class Devices
{
	  // Motor CAN ID/PWM port assignments (1=left-front, 2=left-rear, 3=right-front, 4=right-rear)
	  private static WPI_TalonSRX	LFCanTalon, LRCanTalon, RFCanTalon, RRCanTalon;
	  
	  public static DifferentialDrive	robotDrive;

	  public final static Joystick      gamePad = new Joystick(0);       

	  public final static Compressor	compressor = new Compressor(0);	// Compressor class represents the PCM. There are 2.
	  
	  public final static AnalogInput	pressureSensor = new AnalogInput(0);
	  
	  public final static PowerDistributionPanel	PDP = new PowerDistributionPanel();

	  public final static DriverStation				ds = DriverStation.getInstance();

	  public static NavX				navx;
	  
	  public final static ValveDA             gearShiftValve = new ValveDA(0);
	  public static SRXMagneticEncoderRelative         leftEncoder, rightEncoder;
	 
	  
	  // Create RobotDrive object for CAN Talon controllers.
	  
	  public static void InitializeCANTalonDrive()
	  {
		  Util.consoleLog();

		  LFCanTalon = new WPI_TalonSRX(1);
		  LRCanTalon = new WPI_TalonSRX(2);
		  RFCanTalon = new WPI_TalonSRX(3);
		  RRCanTalon = new WPI_TalonSRX(4);
		  // Magnetic Encoders
		  
	      // Initialize CAN Talons and write status to log so we can verify
	      // all the Talons are connected.
	      InitializeCANTalon(LFCanTalon);
	      InitializeCANTalon(LRCanTalon);
	      InitializeCANTalon(RFCanTalon);
	      InitializeCANTalon(RRCanTalon);
	      
	      // Configure CAN Talons with correct inversions.
	      LFCanTalon.setInverted(true);
		  LRCanTalon.setInverted(true);
		  
		  RFCanTalon.setInverted(true);
		  RRCanTalon.setInverted(true);
	      
	      // Turn on brake mode for CAN Talons.
	      SetCANTalonBrakeMode(true);
	      
	      // Configure SRX encoders
	      // Use 5.8 as the Wheel Diameter
		  rightEncoder = new SRXMagneticEncoderRelative(RRCanTalon, 5.8);
		  leftEncoder = new SRXMagneticEncoderRelative(LRCanTalon, 5.8);
		  
		  leftEncoder.setInverted(true);
		  leftEncoder.setMaxPeriod(1);
		  rightEncoder.setMaxPeriod(1);
		  
		  leftEncoder.setPIDSourceType(PIDSourceType.kRate);
		  leftEncoder.setPIDRateType(PIDRateType.RPM);
		  
		  rightEncoder.setPIDSourceType(PIDSourceType.kRate);
		  rightEncoder.setPIDRateType(PIDRateType.RPM);
	      
	      // Setup the SpeedControllerGroups for the left and right set of motors.
	      SpeedControllerGroup LeftGroup = new SpeedControllerGroup(LFCanTalon,LRCanTalon);
		  SpeedControllerGroup RightGroup = new SpeedControllerGroup(RFCanTalon,RRCanTalon);
		  
		  
		  // Motors In the RF and LF are Unplugged
		  robotDrive = new DifferentialDrive(LRCanTalon, RRCanTalon);
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
	  
	  
	  // Return voltage and current draw for each CAN Talon.
	  
	  public static String GetCANTalonStatus()
	  {
		  return String.format("%.1f/%.1f  %.1f/%.1f  %.1f/%.1f  %.1f/%.1f", 
				  LFCanTalon.getMotorOutputVoltage(), LFCanTalon.getOutputCurrent(),
				  LRCanTalon.getMotorOutputVoltage(), LRCanTalon.getOutputCurrent(),
				  RFCanTalon.getMotorOutputVoltage(), RFCanTalon.getOutputCurrent(),
				  RRCanTalon.getMotorOutputVoltage(), RRCanTalon.getOutputCurrent());
	  }

}
