package Team4450.Robot11;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lift
{
	private final Robot	robot;
	private boolean		climbWinch;
	
	public Lift(Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
		
		updateDS();
	}

	public void dispose()
	{
		Util.consoleLog();
	}
	
	public boolean isClimbWinchSelected()
	{
		Util.consoleLog();
		
		return climbWinch;
	}
	
	public void selectClimbWinch()
	{
		Util.consoleLog();

		setWinchPower(0);

		climbWinch = true;
		
		updateDS();
	}
	
	public void selectLiftWinch()
	{
		Util.consoleLog();

		climbWinch = false;
		
		updateDS();
	}
	
	public void updateDS()
	{
		SmartDashboard.putBoolean("Winch", climbWinch);
	}
	
	public void setWinchPower(double power)
	{
		if (climbWinch)
			Devices.climbWinch.set(power);
		else
			Devices.liftWinch.set(power);
	}
	
	public void releaseForks()
	{
		Util.consoleLog();

	}
	
	/**
	 * Start thread to move cube lift to the desired height as measured
	 * by encoder count.
	 * @param count Target encoder count (height).
	 */
	public void setHeight(int count)
	{
		Util.consoleLog("%d", count);
	}
}
