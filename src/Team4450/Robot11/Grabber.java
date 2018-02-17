package Team4450.Robot11;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Grabber
{
	private final Robot	robot;

	public Grabber(Robot robot)
	{
		Util.consoleLog();
				
		this.robot = robot;

		SmartDashboard.putBoolean("Deployed", false);
		SmartDashboard.putBoolean("Intake", false);
		SmartDashboard.putBoolean("Spit", false);
		SmartDashboard.putBoolean("Grabber", false);
	  
		stop();
		
		retract();
		
		open();
	}

	public void dispose()
	{
		Util.consoleLog();
	}
		
	public void open()
	{
		Util.consoleLog();

		SmartDashboard.putBoolean("Grabber", true);

		Devices.grabberValve.SetA();
	}
	
	public void close()
	{
		Util.consoleLog();
		
		SmartDashboard.putBoolean("Grabber", false);

		Devices.grabberValve.SetB();
	}
	
	public void retract()
	{
		Util.consoleLog();
		
		SmartDashboard.putBoolean("Deployed", false);

		Devices.grabberValve.SetA();
	}
	
	public void extend()
	{
		Util.consoleLog();
		
		SmartDashboard.putBoolean("Deployed", true);

		Devices.grabberValve.SetB();
	}
	
	public void motorsIn(double power)
	{
		Util.consoleLog("%.2f", power);
		
		SmartDashboard.putBoolean("Intake", true);
		SmartDashboard.putBoolean("Spit", false);

		Devices.grabberGroup.set(power);
	}

	public void motorsOut(double power)
	{
		Util.consoleLog("%.2f", power);
		
		SmartDashboard.putBoolean("Intake", false);
		SmartDashboard.putBoolean("Spit", true);

		Devices.grabberGroup.set(-power);
	}
	
	public void stop()
	{
		Util.consoleLog();
		
		SmartDashboard.putBoolean("Intake", false);
		SmartDashboard.putBoolean("Spit", false);

		Devices.grabberGroup.set(0);
	}
}

