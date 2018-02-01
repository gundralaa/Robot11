package Team4450.Robot11;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;

public class Lift
{
	private final Robot	robot;
	
	public Lift(Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
	}

	public void dispose()
	{
		Util.consoleLog();
	}
	
	/**
	 * Start thread to move lift to the desired height as measured
	 * by encoder count.
	 * @param count Target encoder count (height).
	 */
	public void setHeight(int count)
	{
		Util.consoleLog("%d", count);
	}
}
