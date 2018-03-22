
package Team4450.Robot11;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous
{
	private final Robot		robot;
	private final int		program = (int) SmartDashboard.getNumber("AutoProgramSelect",0);
	private PlateStates		plateState;
	private final Lift		lift;
	private final Grabber	grabber;
	private final GearBox	gearBox;
	private final double	spitPower = .50;
	
	public static double	pValue = -6;
	public static double	iValue = 30;
	public static double	dValue = 900;
	
	Autonomous(Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
		
		gearBox = new GearBox(robot);
		
		lift = new Lift(robot);
		
		grabber = new Grabber(robot);
	}

	public void dispose()
	{
		Util.consoleLog();
		
		if (gearBox != null) gearBox.dispose();
		if (lift != null) lift.dispose();
		if (grabber != null) grabber.dispose();
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
		
		// Get the randomized scoring plate state.
		try
		{
			plateState = PlateStates.valueOf(robot.gameMessage);
		}
		catch (Exception e) { plateState = PlateStates.UNDEFINED; }
		
		Devices.robotDrive.setSafetyEnabled(false);

		Util.consoleLog("1");
		// Initialize encoder.
		Devices.wheelEncoder.reset();
        
		Util.consoleLog("2");
       // Set NavX to heading 0.
		Devices.navx.resetYaw();

		Util.consoleLog("3");
		Devices.navx.setHeading(0);

		Util.consoleLog("4");
		// Determine which auto program to run as indicated by driver station.
		switch (program)
		{
			case 0:		// No auto program.
				break;

			case 1:		// Start outside (either side) no scoring.
				startOutsideNoScore();
				break;
			
			case 2:		// Start center no scoring.
				startCenterNoScore();
				break;
			
			case 3:		// Start center score cube.
				startCenterScore();
				break;
			
			case 4:		// Start left outside score cube.
				startOutsideScore(true);
				break;

			case 5:		// Start right outside score cube.
				startOutsideScore(false);
				break;
				
			case 6:		// Start center score cube.
				startCenterScore2();
				break;
				
			case 7:		// Start center score cube.
				startCenterScore3();
				break;
		}
		
		Util.consoleLog("end");
	}

	// Start from left or right and just drive across the line.
	private void startOutsideNoScore()
	{
		Util.consoleLog();
		
		autoDrive(-.50, 2490, true);	// 1606
	}

	// Start from center (offset right). Move forward a bit to get off the wall. 
	// Turn right 45, drive forward to break the line and stop.
 
	private void startCenterNoScore()
	{
		Util.consoleLog();
		
		autoDrive(-.30, 1970, true);	// 1270
		
		//autoRotate(.50, -45);
		
		//autoDrive(.50, 1000, true);
	}

	// Start from center (offset right). Evaluate game information. Determine which switch we
	// should score on. Navigate by moving a bit off the wall, turn 90 in correct direction, 
	// drive forward correct amount, turn 90 in correct direction, raise cube, drive forward 
	// to the switch wall, dump cube.

	private void startCenterScore()
	{
		Util.consoleLog(plateState.toString());
		
		// close, deploy grabber then lift.
		
		grabber.close();
		grabber.deploy();
		Timer.delay(0.5);
		
		if (robot.isClone)
			lift.setHeight(9100);
		else
			lift.setHeight(7900);
		
		autoDrive(-.40, 925, true);		// 596
		
		switch (plateState)
		{
			case UNDEFINED:
				startCenterNoScore();
				return;
				
			case LLL: case LRL:
				autoRotate(.50, 90);
				autoDrive(-.60, 928, true);		// 663
				autoRotate(-.50, 90);
				autoDrive(-.40, 880, true);		// 567
				break;
				
			case RRR: case RLR:
				autoRotate(-.50, 90);
				autoDrive(-.60, 900, true);		// 857
				autoRotate(.50, 90);
				autoDrive(-.40, 880, true);		// 567
				break;
		}
		
		// Dump cube.
		
		grabber.spit(spitPower);
		
		// Back up and drop the lift.
		
		lift.setHeight(-1);
//		autoDrive(.30, 500, true);
//		lift.setHeight(0);
//		Timer.delay(3.0);
	}

	private void startCenterScore2()
	{
		Util.consoleLog(plateState.toString());
		
		// close, deploy grabber then lift.
		
		grabber.close();
		grabber.deploy();
		Timer.delay(0.5);
		
		if (robot.isClone)
			lift.setHeight(9100);
		else
			lift.setHeight(7900);
		
		autoDrive(-.40, 100, true);		// 596
		
		switch (plateState)
		{
			case UNDEFINED:
				startCenterNoScore();
				return;
				
			case LLL: case LRL:
				autoRotate(.50, 24);
				autoDrive(-.60, 2100, true);	// 663
				//autoRotate(-.50, 90);
				//autoDrive(-.40, 880, true);		// 567
				break;
				
			case RRR: case RLR:
				autoRotate(-.50, 16);
				autoDrive(-.60, 1900, true);	// 857
				//autoRotate(.50, 12);
				//autoDrive(-.40, 880, true);		// 567
				break;
		}
		
		// Dump cube.
		
		grabber.spit(spitPower);
		
		// Back up and drop the lift.
		
		//lift.setHeight(-1);
//		autoDrive(.30, 500, true);
//		lift.setHeight(0);
//		Timer.delay(3.0);
	}

	private void startCenterScore3()
	{
		Util.consoleLog(plateState.toString());
		
		// close, deploy grabber then lift.
		
		grabber.close();
		grabber.deploy();
		Timer.delay(0.5);
		
		if (robot.isClone)
			lift.setHeight(9100);
		else
			lift.setHeight(7900);
		
		switch (plateState)
		{
			case UNDEFINED:
				startCenterNoScore();
				return;
				
			case LLL: case LRL:
//				autoSCurve(-.50, SmartDashboard.getNumber("PValue", 6),
//							(int) SmartDashboard.getNumber("IValue", 30),
//							(int) SmartDashboard.getNumber("DValue", 900));
				autoSCurve(-.50, 6, 30,	900);

				break;
				
			case RRR: case RLR:
//				autoSCurve(-.50, SmartDashboard.getNumber("PValue", -6),
//						(int) SmartDashboard.getNumber("IValue", 30),
//						(int) SmartDashboard.getNumber("DValue", 900));
				autoSCurve(-.50, -6, 30,	900);

				break;
		}
		
		// Dump cube.
		
		grabber.spit(spitPower);
		
		// Back up and drop the lift.
		
		//lift.setHeight(-1);
//		autoDrive(.30, 500, true);
//		lift.setHeight(0);
//		Timer.delay(3.0);
	}

	// Start left or right. Evaluate game information. Determine if we should score on the switch, 
	// scale, or not at all. For not at all, drive forward until aligned with the platform area, 
	// turn right 90, drive forward into the platform area as far as we can get toward the scale on 
	// opposite side of the field. For score scale drive forward raising cube until aligned with scale
	// front, turn right 90, drive to scale drop position, drop cube. For score switch drive forward
	// raising cube until aligned with switch front, turn right 90, drive to switch wall, drop cube.

	private void startOutsideScore(boolean startingLeft)
	{
		Util.consoleLog("start left=%b, plate=%s", startingLeft, plateState.toString());
		
		// close, deploy grabber then lift.
		
		grabber.close();
		grabber.deploy();
		Timer.delay(0.5);
		
		if (robot.isClone)
			lift.setHeight(9100);
		else
			lift.setHeight(7900);
		
		if (startingLeft) 
		{
			switch (plateState)
			{
				case UNDEFINED:
					return;
					
				//case LLL: case RLR:	// Scale available.
					//autoDrive(.50, 1000, true);
					//autoRotate(.50, -90);
					//autoDrive(.50, 1000, true);
					//break;
					
				case RRR:  case RLR:	// No plate available.
					autoDrive(-.50, 2500, true);	// 4600/2967
					//autoRotate(-.50, 90);
					//autoDrive(-.50, 1470, true);	// 948
					
					// Drop the lift.
					lift.setHeight(-1);
//					lift.setHeight(0);
//					Timer.delay(3.0);
					return;
					
				case LRL: case LLL:		// Switch available.
					autoDrive(-.50, 3180, true);	// 2051
					autoRotate(-.50, 90);
					autoDrive(-.30, 320, true);		// 206
					break;
			}
		}
		else
		{
			switch (plateState)
			{
				case UNDEFINED:
					return;
					
				case LLL: case LRL:	// No plate available.
					autoDrive(-.50, 2500, true);	// 4600/2967
					//autoRotate(.50, 90);
					//autoDrive(-.50, 1470, true);	// 948
					
					// Drop the lift.
					lift.setHeight(-1);
					//Timer.delay(3.0);
					return;
					
//				case RRR: case LRL:	// Scale available.
//					autoDrive(.50, 1000, true);
//					autoRotate(.50, 90);
//					autoDrive(.50, 1000, true);
//					break;
//					
				case RLR: case RRR:	// Switch available.
					autoDrive(-.50, 3180, true);	// 2051
					autoRotate(.50, 90);
					autoDrive(-.30, 320, true);		// 206s
					break;
			}
		}
		
		// Dump cube.
		
		grabber.spit(spitPower);
		
		// Back up and drop the lift.
		
		lift.setHeight(-1);
//		autoDrive(.30, 500, true);
//		lift.setHeight(0);
//		Timer.delay(3.0);
	}
	
	// Auto drive straight in set direction and power for specified encoder count. Stops
	// with or without brakes on CAN bus drive system. Uses NavX to go straight.
	
	private void autoDrive(double power, int encoderCounts, boolean enableBrakes)
	{
		int		angle;
		double	gain = .05;
		
		Util.consoleLog("pwr=%.2f, count=%d, brakes=%b", power, encoderCounts, enableBrakes);

		Devices.SetCANTalonBrakeMode(enableBrakes);

		Devices.wheelEncoder.reset();
		Devices.wheelEncoder2.reset();
		
		Devices.navx.resetYaw();
		
		while (isAutoActive() && Math.abs(Devices.wheelEncoder.get()) < encoderCounts) 
		{
			LCD.printLine(4, "wheel encoder=%d  winch encoder=%d", Devices.wheelEncoder.get(), Devices.winchEncoder.get());
			
			// Angle is negative if robot veering left, positive if veering right when going forward.
			// It is opposite when going backward. Note that for this robot, - power means forward and
			// + power means backward.
			
			angle = (int) Devices.navx.getYaw();
			
			// Invert angle for backwards.
			
			if (power > 0) angle = -angle;

			LCD.printLine(5, "angle=%d", angle);
			
			//Util.consoleLog("angle=%d", angle);
			
			// Note we invert sign on the angle because we want the robot to turn in the opposite
			// direction than it is currently going to correct it. So a + angle says robot is veering
			// right so we set the turn value to - because - is a turn left which corrects our right
			// drift.
			
			// Update: The new curvatureDrive function expects the power to be + for forward motion.
			// Since our power value is - for forward, we do not invert the sign of the angle like
			// we did with previous drive functions. This code base should be updated to fix the
			// Y axis sign to be + for forward. This would make more sense and simplify understanding
			// the code and would match what curvatureDrive expects. Will wait on that until after
			// 2018 season. After fixing that, the angle would again need to be inverted.
			
			Devices.robotDrive.curvatureDrive(power, angle * gain, false);
			
			Timer.delay(.020);
		}

		Devices.robotDrive.tankDrive(0, 0, true);				
		
		Util.consoleLog("end: actual count=%d", Math.abs(Devices.wheelEncoder.get()));
	}
	
	// Auto rotate left or right the specified angle. Left/right from robots forward view.
	// Turn right, power is -
	// Turn left, power is +
	// angle of rotation is always +.
	
	private void autoRotate(double power, int angle)
	{
		Util.consoleLog("pwr=%.2f  angle=%d", power, angle);
		
		Devices.navx.resetYaw();
		
		Devices.robotDrive.tankDrive(power, -power);

		while (isAutoActive() && Math.abs((int) Devices.navx.getYaw()) < angle) {Timer.delay(.020);} 
		
		Devices.robotDrive.tankDrive(0, 0);
	}
	
	private void autoSCurve(double power, double curve, int targetAngle, int straightEncoderCounts)
	{
		double	gain = .05;
		
		Util.consoleLog("pwr=%.2f  curve=%.2f  angle=%d  counts=%d", power, curve, targetAngle, straightEncoderCounts);
		
		// We start out driving in a curve until we have turned the desired angle.
		// Then we drive straight the desired distance then curve back to starting
		// angle. Curve is - for right, + for left.
		
		Devices.robotDrive.curvatureDrive(power, curve * gain, false);
		
		while (isAutoActive() && Math.abs((int) Devices.navx.getYaw()) < targetAngle) 
		{
			LCD.printLine(6, "angle=%.2f", Devices.navx.getYaw());
			Util.consoleLog("angle=%.2f", Devices.navx.getYaw());
			Timer.delay(.020);
		}
		
		autoDrive(power, straightEncoderCounts, false);

		Devices.navx.resetYaw();
		
		Devices.robotDrive.curvatureDrive(power, -curve * gain, false);
		
		while (isAutoActive() && Math.abs((int) Devices.navx.getYaw()) < targetAngle) 
		{
			LCD.printLine(6, "angle=%.2f", Devices.navx.getYaw());
			Util.consoleLog("angle=%.2f", Devices.navx.getYaw());
			Timer.delay(.020);
		}

		Devices.robotDrive.tankDrive(0, 0, true);
	}
	
	private enum PlateStates
	{
		UNDEFINED,
		LLL,
		RRR,
		LRL,
		RLR;
 	}
}