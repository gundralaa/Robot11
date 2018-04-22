/**
 * 2018 competition robot code.
 *
 * For Robot "Odyssey" built for FRC game "FIRST POWER UP".
*/

package Team4450.Robot11;

import java.util.Properties;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file.
 */

public class Robot extends SampleRobot 
{
  static final String  	PROGRAM_NAME = "SWF11.3-04.16.18-01";

  public Properties		robotProperties;
  
  public static boolean		isClone = false, isComp = false;
    	
  DriverStation.Alliance	alliance;
  int                       location, matchNumber;
  String					eventName, gameMessage;
    
  Thread               	monitorBatteryThread, monitorPDPThread;
  MonitorCompressor		monitorCompressorThread;
  CameraFeed			cameraThread;
      
  // Constructor.
  
  public Robot()
  {	
	// Set up our custom logger.
	 
	try
	{
		Util.CustomLogger.setup();
    }
    catch (Exception e) {Util.logException(e);}
      
    try
    {
    	Util.consoleLog(PROGRAM_NAME);

    	Util.consoleLog("RobotLib=%s", LibraryVersion.version);
    	
    	SmartDashboard.putString("Sean's Debug Table/ProgramInfo/RobotLib Version", LibraryVersion.version);
    }
    catch (Exception e) {Util.logException(e);}
  }
    
  // Initialization, called at class start up.
  
  public void robotInit()
  {
   	try
    {
   		Util.consoleLog();

   		LCD.clearAll();
   		LCD.printLine(1, "Mode: RobotInit");
   		
   		// Read properties file from RoboRio "disk".
      
   		robotProperties = Util.readProperties();
      
   		// Is this the competition or clone robot?
   		
		if (robotProperties.getProperty("RobotId").equals("comp"))
			isComp = true;
		else
			isClone = true;

   		SmartDashboard.putString("Program", PROGRAM_NAME);
   		
   		SmartDashboard.putBoolean("CompressorEnabled", Boolean.parseBoolean(robotProperties.getProperty("CompressorEnabledByDefault")));

   		// Reset PDB & PCM sticky faults.
      
   		Devices.PDP.clearStickyFaults();
   		Devices.compressor.clearAllPCMStickyFaults();
   		
   		SmartDashboard.putData("Sean's Debug Table/ControlSystem/PDP", Devices.PDP);
   		SmartDashboard.putData("Sean's Debug Table/ControlSystem/Compressor", Devices.compressor);
   		
   		// Configure motor controllers and RobotDrive.
   		
   		Devices.InitializeCANTalonDrive();
		
   		Devices.robotDrive.stopMotor();
   		Devices.robotDrive.setSafetyEnabled(false);
   		Devices.robotDrive.setExpiration(0.1);
   		
   		Devices.driveEncoder1.setReverseDirection(false);
   		Devices.driveEncoder2.setReverseDirection(false);
   		
   		Devices.winchEncoder.setReverseDirection(!isComp);
   		
   		// Create NavX object here so it has time to calibrate before we
   		// use it. Takes 10 seconds. Must appear before CamerFeed is created.
   		
   		Devices.navx = NavX.getInstance(NavX.PortType.SPI);
   		
   		Devices.navx.dumpValuesToNetworkTables();

   		// Start the battery, compressor, PDP and camera feed monitoring Tasks.

   		monitorBatteryThread = MonitorBattery.getInstance();
   		monitorBatteryThread.start();

   		monitorCompressorThread = MonitorCompressor.getInstance(Devices.pressureSensor);
   		monitorCompressorThread.setDelay(1.0);
   		monitorCompressorThread.SetLowPressureAlarm(50);
   		monitorCompressorThread.start();
   		
   		//monitorPDPThread = MonitorPDP.getInstance(ds, PDP);
   		//monitorPDPThread.start();

   		// Start camera server using our class for usb cameras.
      
       	cameraThread = CameraFeed.getInstance(); 
       	cameraThread.start();
       	
       	Devices.updateNetworkTables();
   		
   		Util.consoleLog("end");
    }
    catch (Exception e) {Util.logException(e);}
  }
  
  // Called when robot is disabled.
  
  public void disabled()
  {
	  try
	  {
		  Util.consoleLog();

		  LCD.printLine(1, "Mode: Disabled");

		  // Reset driver station LEDs.

		  SmartDashboard.putBoolean("Disabled", true);
		  SmartDashboard.putBoolean("Auto Mode", false);
		  SmartDashboard.putBoolean("Teleop Mode", false);
		  SmartDashboard.putBoolean("FMS", Devices.ds.isFMSAttached());
		  SmartDashboard.putBoolean("AutoTarget", false);
		  SmartDashboard.putBoolean("TargetLocked", false);
		  SmartDashboard.putBoolean("Overload", false);
		  SmartDashboard.putNumber("AirPressure", 0);
		  
		  Util.consoleLog("end");
	  }
	  catch (Exception e) {Util.logException(e);}
  }
  
  // Called at the start of Autonomous period.
  
  public void autonomous() 
  {
      try
      {
    	  Util.consoleLog();

    	  LCD.clearAll();
    	  LCD.printLine(1, "Mode: Autonomous");
            
    	  SmartDashboard.putBoolean("Disabled", false);
    	  SmartDashboard.putBoolean("Auto Mode", true);
        
    	  // Make available the alliance (red/blue) and staring position as
    	  // set on the driver station or FMS.
        
    	  alliance = Devices.ds.getAlliance();
    	  location = Devices.ds.getLocation();
    	  eventName = Devices.ds.getEventName();
    	  matchNumber = Devices.ds.getMatchNumber();
    	  gameMessage = Devices.ds.getGameSpecificMessage();

    	  // This code turns off the automatic compressor management if requested by DS.
    	  Devices.compressor.setClosedLoopControl(SmartDashboard.getBoolean("CompressorEnabled", true));

    	  // Reset persistent fault flags in control system modules.
    	  Devices.PDP.clearStickyFaults();
    	  Devices.compressor.clearAllPCMStickyFaults();
             
    	  // Start autonomous process contained in the Autonomous class.
        
    	  Autonomous autonomous = new Autonomous(this);
        
    	  autonomous.execute();
        
    	  autonomous.dispose();
    	  
    	  SmartDashboard.putBoolean("Auto Mode", false);
    	  Util.consoleLog("end");
      }
      catch (Exception e) {Util.logException(e);}
  }

  // Called at the start of the teleop period.
  
  public void operatorControl() 
  {
      try
      {
    	  Util.consoleLog();

    	  LCD.clearAll();
      	  LCD.printLine(1, "Mode: Teleop");
            
      	  SmartDashboard.putBoolean("Disabled", false);
      	  SmartDashboard.putBoolean("Teleop Mode", true);
      	  
      	  alliance = Devices.ds.getAlliance();
      	  location = Devices.ds.getLocation();
    	  eventName = Devices.ds.getEventName();
    	  matchNumber = Devices.ds.getMatchNumber();
    	  gameMessage = Devices.ds.getGameSpecificMessage();
        
          Util.consoleLog("Alliance=%s, Location=%d, FMS=%b event=%s match=%d matchType=%s msg=%s", 
        		  		   alliance.name(), location, Devices.ds.isFMSAttached(), eventName, matchNumber, Devices.ds.getMatchType().name(), gameMessage);

    	  // Reset persistent fault flags in control system modules.
          Devices.PDP.clearStickyFaults();
          Devices.compressor.clearAllPCMStickyFaults();

          // This code turns off the automatic compressor management if requested by DS.
          Devices.compressor.setClosedLoopControl(SmartDashboard.getBoolean("CompressorEnabled", true));
        
          // Start operator control process contained in the Teleop class.
        
          Teleop teleOp = new Teleop(this);
       
          teleOp.execute();
        
          teleOp.dispose();
        	
          Util.consoleLog("end");
       }
       catch (Exception e) {Util.logException(e);} 
  }
    
  public void test() 
  {
  }

  // Start usb camera server for single camera.
  
  public void StartUSBCameraServer(String cameraName, int device)
  {
	  Util.consoleLog("%s:%d", cameraName, device);

      CameraServer.getInstance().startAutomaticCapture(cameraName, device);
  }
}
