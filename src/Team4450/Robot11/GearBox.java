/**
 * Manage gear box shifting.
 */
package Team4450.Robot11;

import Team4450.Lib.*;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearBox
{
	private Robot		robot;
	private boolean		lowSpeed,  highSpeed;
	
	public GearBox	(Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
		
		lowSpeed();
	}
	
	public void dispose()
	{
		Util.consoleLog();
	}
	
	private void displayStatus()
	{
		Util.consoleLog("low=%b, high=%b", lowSpeed, highSpeed);
		
		SmartDashboard.putBoolean("Low", lowSpeed);
		SmartDashboard.putBoolean("High", highSpeed);
	}

	/**
	 * Set gear boxes into low speed. Pushes the dog ring to the inside.
	 */
	public void lowSpeed()
	{
		Util.consoleLog();

		highSpeed = false;

		Devices.highLowValve.SetA();
		
		lowSpeed = true;
		
		displayStatus();
	}

	/**
	 * Set gear boxes into high speed. Pushes the dog ring to the outside.
	 */
	public void highSpeed()
	{
		Util.consoleLog();

		lowSpeed = false;
		
		Devices.highLowValve.SetB();
		
		highSpeed = true;
		
		displayStatus();
	}

	/**
	 * Return low speed state.
	 * @return True if low speed.
	 */
	public boolean isLowSpeed()
	{
		return lowSpeed;
	}
	
	/**
	 * Return high speed state.
	 * @return True if high speed.
	 */
	public boolean isHighSpeed()
	{
		return highSpeed;
	}
}
