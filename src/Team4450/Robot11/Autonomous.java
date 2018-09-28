
package Team4450.Robot11;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;
import jaci.pathfinder.*;

/*
 * Main Method of this testing robot
 * Use Pathfinder library to create adaptive way points usable for
 * Competition
 * 
 * Use Vision to Autonomously find blocks and navigate using
 * Waypoints or heading values
 */

public class Autonomous
{
	private final Robot	robot;
	private final int	program = (int) SmartDashboard.getNumber("AutoProgramSelect",0);
	
	Waypoint [] points = {
			new Waypoint(0,0,0),
			new Waypoint(1,0,0)
	};
	
	Autonomous(Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;		
	}

	public void dispose()
	{
		Util.consoleLog();
	}
	
	private boolean isAutoActive()
	{
		return robot.isEnabled() && robot.isAutonomous();
	}

	public void execute()
	{
		Util.consoleLog("Alliance=%s, Location=%d, Program=%d, FMS=%b, msg=%s", robot.alliance.name(), robot.location, program, 
				Devices.ds.isFMSAttached(), robot.gameMessage);
		LCD.printLine(2, "Alliance=%s, Location=%d, FMS=%b, Program=%d, msg=%s", robot.alliance.name(), robot.location, 
				Devices.ds.isFMSAttached(), program, robot.gameMessage);

		Devices.robotDrive.setSafetyEnabled(false);
		
		// Encoders Reset.
		Devices.rightEncoder.reset();
	    Devices.leftEncoder.reset();
				
		// Set NavX yaw tracking to 0.
	    Devices.navx.resetYaw();

		Devices.navx.getAHRS().resetDisplacement();
				
		//Reset Heading Value
		Devices.navx.setHeading(0);
		
		// Target Heading Change
		Devices.navx.setTargetHeading(0);
		
        // Wait to start motors so gyro will be zero before first movement.
        Timer.delay(.50);

		switch (program)
		{
			case 0:		// No auto program.
				break;
			
			case 1:
				wayPointAutonTest();
				break;
		}
		
		Util.consoleLog("end");
	}
	
	private void wayPointAutonTest() {
		// Use Pathfinder Library
		
		
		// Trajectory Configuration Values
		double max_velocity = 1.0;	//1.7;
		double max_acceleration = 0.3;
		double max_jerk = 60.0;
		double time_step = .05;
		
		// Create a Trajectory Config Object
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, 
														 Trajectory.Config.SAMPLES_HIGH, 
														 time_step, 
														 max_velocity, 
														 max_acceleration, 
														 max_jerk);
		// Generate the trajectory
		// Using Waypoints define above
		Trajectory trajectory = Pathfinder.generate(points, config);
		
		// The distance in meters between the wheel sides
		double distanceBetweenLR = Util.inchesToMeters(28);
		double wheelDiameter = Util.inchesToMeters(5.8);
		
		// Allow to divide between the wheel sides
		TankModifier modification = new TankModifier(trajectory);
		modification.modify(distanceBetweenLR);
		
		Trajectory leftTrajectory = modification.getLeftTrajectory();
		Trajectory rightTrajectory = modification.getRightTrajectory();
		
		// Create encoder follower for each encoder.
		EncoderFollower left = new EncoderFollower(leftTrajectory);
		EncoderFollower right = new EncoderFollower(rightTrajectory);
		
		// 4096 Ticks per Revolution
		left.configureEncoder(Devices.leftEncoder.get(), 4096,wheelDiameter);
		right.configureEncoder(Devices.rightEncoder.get(), 4096, wheelDiameter);
		
		// Configure PID Controller
		left.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0);	
		right.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0);
		
		while (isAutoActive() && !left.isFinished())
		{
			double lPower = left.calculate(Devices.leftEncoder.get());
			double rPower = right.calculate(Devices.rightEncoder.get());

			double gyro_heading = Devices.navx.getHeadingR();			// degrees
			double desired_heading = Pathfinder.r2d(left.getHeading()); // Should also be in degrees

			double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
			double turn = -0.02 * angleDifference;

			lPower = Util.clampValue(lPower + turn, 1.0);
			rPower = Util.clampValue(rPower - turn, 1.0);
			
			Util.consoleLog("le=%d lp=%.2f  re=%d rp=%.2f  dhdg=%.0f  hdg=%.0f ad=%.2f  turn=%.2f  time=%.3f", 
							Devices.leftEncoder.get(), lPower, Devices.rightEncoder.get(), rPower, 
							desired_heading, gyro_heading, angleDifference, turn,  Util.getElaspedTime());
			
			//turn = 0;
			
			Devices.robotDrive.tankDrive(lPower, rPower);
			
			Timer.delay(time_step);
		}
		
		Devices.robotDrive.stopMotor();
		
	}
	
	
}