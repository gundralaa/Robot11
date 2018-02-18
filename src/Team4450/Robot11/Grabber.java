package Team4450.Robot11;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Grabber
{
	private final Robot	robot;
	private boolean		grabberOpen, grabberDeployed, intake, spit;

	public Grabber(Robot robot)
	{
		Util.consoleLog();
				
		this.robot = robot;
	  
		stop();
		
		retract();
		
		close();
	}

	public void dispose()
	{
		Util.consoleLog();
	}
		
	public void open()
	{
		Util.consoleLog();

		grabberOpen = true;
		
		SmartDashboard.putBoolean("Grabber", grabberOpen);

		Devices.grabberValve.SetA();
	}
	
	public void close()
	{
		Util.consoleLog();
		
		grabberOpen =  false;
		
		Devices.grabberValve.SetB();

		updateDS();
	}
	
	public void retract()
	{
		Util.consoleLog();
		
		grabberDeployed = false;

		Devices.grabberValve.SetA();
		
		updateDS();
	}
	
	public void deploy()
	{
		Util.consoleLog();
		
		grabberDeployed = true;
		
		Devices.grabberValve.SetB();

		updateDS();
	}
	
	public void motorsIn(double power)
	{
		Util.consoleLog("%.2f", power);
		
		intake = true;
		spit = false;

		Devices.grabberGroup.set(power);
		
		updateDS();
	}

	public void motorsOut(double power)
	{
		Util.consoleLog("%.2f", power);
		
		intake = false;
		spit = true;

		Devices.grabberGroup.set(-power);

		updateDS();
	}
	
	public void spit(double power)
	{
		Util.consoleLog("%.2f", power);
		
		motorsOut(power);
		
		Timer.delay(2);
		
		stop();
	}
	
	public void stop()
	{
		Util.consoleLog();
		
		intake = false;
		spit = false;

		Devices.grabberGroup.set(0);
		
		updateDS();
	}
	
	public boolean isOpen()
	{
		return grabberOpen;
	}
	
	public boolean isDeployed()
	{
		return grabberDeployed;
	}
	
	public boolean isIntake()
	{
		return intake;
	}
	
	public boolean isSpit()
	{
		return spit;
	}
	
	private void updateDS()
	{
		SmartDashboard.putBoolean("Grabber", grabberOpen);
		SmartDashboard.putBoolean("Deployed", grabberDeployed);
		SmartDashboard.putBoolean("Intake", intake);
		SmartDashboard.putBoolean("Spit", spit);
	}
}

