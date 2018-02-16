package Team4450.Robot11;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;

public class Grabber
{
	private final Robot	robot;

	public Grabber(Robot robot)
	{
		Util.consoleLog();
				
		this.robot = robot;
		
		this.stop();
		
		this.retract();
		
		this.open();
	}

	public void dispose()
	{
		Util.consoleLog();
	}
		
	public void open()
	{
		Util.consoleLog();
		
		Devices.grabberValve.SetA();
	}
	
	public void close()
	{
		Util.consoleLog();
		
		Devices.grabberValve.SetB();
	}
	
	public void retract()
	{
		Util.consoleLog();
		
		Devices.grabberValve.SetA();
	}
	
	public void extend()
	{
		Util.consoleLog();
		
		Devices.grabberValve.SetB();
	}
	
	public void motorsIn(double power)
	{
		Util.consoleLog("%.2f", power);
		
		Devices.grabberGroup.set(power);
	}

	public void motorsOut(double power)
	{
		Util.consoleLog("%.2f", power);
		
		Devices.grabberGroup.set(-power);
	}
	
	public void stop()
	{
		Util.consoleLog();
		
		Devices.grabberGroup.set(0);
	}
}

