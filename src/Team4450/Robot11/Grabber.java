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

	public void motorsIn(double power)
	{
		Util.consoleLog("%.2f", power);
		
		Devices.intakeMotor1.set(power);
		Devices.intakeMotor2.set(power);
	}

	public void motorsOut(double power)
	{
		Util.consoleLog("%.2f", power);
		
		Devices.intakeMotor1.set(-power);
		Devices.intakeMotor2.set(-power);
	}
	
	public void motorStop()
	{
		Util.consoleLog();
		
		Devices.intakeMotor1.set(0);
		Devices.intakeMotor2.set(0);
	}
}

