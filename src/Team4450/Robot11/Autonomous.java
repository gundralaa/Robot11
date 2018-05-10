
package Team4450.Robot11;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
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

		// Initialize encoder.
		Devices.wheelEncoder.reset();
        
       // Set NavX yaw tracking to 0.
		Devices.navx.resetYaw();

		Devices.navx.getAHRS().resetDisplacement();
		
		// Set heading to initial angle (0 is robot pointed down the field) so
		// NavX class can track which way the robot is pointed during the match.
		Devices.navx.setHeading(0);
		
		// Target heading should be the same.
		Devices.navx.setTargetHeading(0);
		
		// Set Talon ramp rate for smooth acceleration from stop. Determine by observation.
		Devices.SetCANTalonRampRate(1.0);
		
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
				startCenterScoreFast();
				break;
				
			case 7:		// Start center score cube.
				startCenterScoreCurve();
				break;
		}
		
		
		Util.consoleLog("y=%.2f  x=%.2f", Devices.navx.getAHRS().getDisplacementY(), Devices.navx.getAHRS().getDisplacementX());
		
		Util.consoleLog("end");
	}

	// Start from left or right and just drive across the line.
	private void startOutsideNoScore()
	{
		Util.consoleLog();
		
		autoDrive(.50, 2490, true);	// 1606
	}

	// Start from center (offset right). Move forward a bit to get off the wall. 
	// Drive forward to break the line and stop.
 
	private void startCenterNoScore()
	{
		Util.consoleLog();
		
		//autoDrive(.30, 1970, true);	// 1270
		autoDrive3(.50, 1970, true, true, true);
	}

	// Start from center (offset right). Evaluate game information. Determine which switch we
	// should score on. Navigate by moving a bit off the wall, turn 90 in correct direction, 
	// drive forward correct amount, turn 90 in correct direction, raise cube, drive forward 
	// to the switch wall, dump cube.

	private void startCenterScore()
	{
		Util.consoleLog(plateState.toString());
		
		// close, deploy grabber then lift.
		
		//grabber.close();
		//grabber.deploy();
		Timer.delay(0.5);
		
//		if (robot.isClone)
//			lift.setHeight(9100);
//		else
//			lift.setHeight(7900);
		
		autoDrive2(.40, 925, true, true);		// 596
		
		switch (plateState)
		{
			case UNDEFINED:
				startCenterNoScore();
				return;
				
			case LLL: case LRL:
				autoRotate2(.30, 270);
				autoDrive2(.60, 928, true, true);		// 663
				autoRotate2(.30, 0);
				autoDrive2(.60, 880, true, true);		// 567
				break;
				
			case RRR: case RLR:
				autoRotate2(.30, 90);
				autoDrive2(.60, 900, true, true);		// 857
				autoRotate2(.30, 0);
				autoDrive2(.60, 880, true, true);		// 567
				break;
		}
		
		// Dump cube.
		
		//grabber.spit(spitPower);
	}

	private void startCenterScoreFast()
	{
		Util.consoleLog(plateState.toString());
		
		// close, deploy grabber then lift.
		
		//grabber.close();
		//grabber.deploy();
		Timer.delay(0.5);
		
//		if (robot.isClone)
//			lift.setHeight(9100);
//		else
//			lift.setHeight(7900);
		
		autoDrive2(.40, 100, true, true);		// 596
		
		switch (plateState)
		{
			case UNDEFINED:
				startCenterNoScore();
				return;
				
			case LLL: case LRL:
				autoRotate2(.50, 334);				// 26 -.50
				autoDrive2(.60, 2100, true, true);	// 663
				//autoRotate(-.50, 90);
				//autoDrive(.40, 880, true);		// 567
				break;
				
			case RRR: case RLR:
				autoRotate2(.50, 18);
				autoDrive(.60, 1900, true);	// 857
				//autoRotate(.50, 12);
				//autoDrive(.40, 880, true);		// 567
				break;
		}
		
		// Dump cube.
		
		//grabber.spit(spitPower);
		
	}

	private void startCenterScoreCurve()
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
				autoSCurve(.50, -.3, 30, 900);

				break;
				
			case RRR: case RLR:
				autoSCurve(.50, .3, 30, 950);

				break;
		}
		
		// Dump cube.
		
		grabber.spit(spitPower);
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
					autoDrive(.50, 2500, true);	// 4600/2967
					//autoRotate(-.50, 90);
					//autoDrive(-.50, 1470, true);	// 948
					
					// Drop the lift.
					lift.setHeight(-1);
//					lift.setHeight(0);
//					Timer.delay(3.0);
					return;
					
				case LRL: case LLL:		// Switch available.
					autoDrive(.50, 3180, true);	// 2051
					autoRotate(-.50, 90);
					autoDrive(.30, 320, true);		// 206
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
					autoDrive(.50, 2500, true);	// 4600/2967
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
					autoDrive(.50, 3180, true);	// 2051
					autoRotate(.50, 90);
					autoDrive(.30, 320, true);		// 206s
					break;
			}
		}
		
		// Dump cube.
		
		grabber.spit(spitPower);
		
	}
	
	/**
	 * Auto drive straight in set direction and power for specified encoder count. Stops
	 * with or without brakes on CAN bus drive system. Uses NavX yaw to drive straight.
	 * @param power Speed, + is forward.
	 * @param encoderCounts encoder counts to travel.
	 * @param enableBrakes True to enable brakes.
	 */
	private void autoDrive(double power, int encoderCounts, boolean enableBrakes)
	{
		int		angle;
		double	gain = .05;
		int		error = 0;
		double	power2 = 0, pFactor, kP = .002, minPower = .10;

		// Min power is determined experimentally for each robot as the lowest power that
		// will move the robot. We don't want the pid reduction in power at the end of
		// the drive to fall below this level and cause the drive to stall before done.
		
		Util.consoleLog("pwr=%.2f  count=%d  brakes=%b", power, encoderCounts, enableBrakes);

		Devices.SetCANTalonBrakeMode(enableBrakes);

		Devices.wheelEncoder.reset();
		Devices.wheelEncoder2.reset();
		
		if (robot.isClone) Timer.delay(0.3);
		
		Util.consoleLog("before reset=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
			
		//Devices.navx.resetYaw();
		
		Util.consoleLog("after reset1=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
		
		Devices.navx.resetYawWait(1, 500);
		
		Util.consoleLog("after reset2=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
		
		while (isAutoActive() && Math.abs(Devices.wheelEncoder.get()) < encoderCounts) 
		{
			LCD.printLine(4, "wheel encoder=%d  winch encoder=%d", Devices.wheelEncoder.get(), Devices.winchEncoder.get());

			// Quick and dirty PID control to reduce power as we approach target encoder counts.
			error = encoderCounts - Math.abs(Devices.wheelEncoder.get());
			pFactor = error * kP;
			power2 = Util.clampValue(power * pFactor, minPower, power);
			Util.consoleLog("error=%d pfactor=%.2f power2=%.2f", error, pFactor, power2);

			// Angle is negative if robot veering left, positive if veering right when going forward.
			// It is opposite when going backward. Note that for this robot, - power means forward and
			// + power means backward.
			
			angle = (int) Devices.navx.getYaw();
			
			// Invert angle for backwards.
	
			if (power < 0) angle = -angle;

			LCD.printLine(5, "angle=%d", angle);
			
			Util.consoleLog("angle=%d  hdg=%.2f", angle, Devices.navx.getHeading());
			
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
			
			// Done in this branch for testing 4-23-18.
			
			Devices.robotDrive.curvatureDrive(power2, Util.clampValue(-angle * gain, 1.0), false);
			
			Timer.delay(.010);
		}

		Devices.robotDrive.tankDrive(0, 0);				
		
		Util.consoleLog("end: actual count=%d", Math.abs(Devices.wheelEncoder.get()));
	}
	
	private void autoDrive2(double power, int encoderCounts, boolean enableBrakes, boolean usePid)
	{
		double	angle, gain = .05;
		int		error = 0;
		double	power2 = 0, pFactor, kP = .002, minPower = .15;

		// Min power is determined experimentally for each robot as the lowest power that
		// will move the robot. We don't want the pid reduction in power at the end of
		// the drive to fall below this level and cause the drive to stall before done.
		
		Util.consoleLog("pwr=%.2f  count=%d  brakes=%b  pid=%b", power, encoderCounts, enableBrakes, usePid);

		Devices.SetCANTalonBrakeMode(enableBrakes);

		Devices.wheelEncoder.reset();
		Devices.wheelEncoder2.reset();
		
		if (robot.isClone) Timer.delay(0.3);
		
		while (isAutoActive() && Math.abs(Devices.wheelEncoder.get()) < encoderCounts) 
		{
			LCD.printLine(4, "wheel encoder=%d  winch encoder=%d", Devices.wheelEncoder.get(), Devices.winchEncoder.get());

			// Quick and dirty PID control to reduce power as we approach target encoder counts.
			
			if (usePid)
			{
				error = encoderCounts - Math.abs(Devices.wheelEncoder.get());
				pFactor = error * kP;
				power2 = Util.clampValue(power * pFactor, minPower, power);
				Util.consoleLog("error=%d pfactor=%.2f power2=%.2f", error, pFactor, power2);
			}
			else
				power2 = power;

			// Angle is negative if robot veering left, positive if veering right when going forward.
			// It is opposite when going backward. Note that for this robot, + power means forward and
			// - power means backward.
			
			angle = Devices.navx.getHeadingYaw();
			
			// Invert yaw angle for backwards.
	
			if (power < 0) angle = -angle;

			LCD.printLine(5, "angle=%.2f", angle);
			
			Util.consoleLog("angle=%.2f  hdg=%.2f", angle, Devices.navx.getHeading());
			
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
			
			// Done in this branch for testing 4-23-18.
			
			Devices.robotDrive.curvatureDrive(power2, Util.clampValue(-angle * gain, 1.0), false);
			
			Timer.delay(.010);
		}

		Devices.robotDrive.stopMotor();	//.tankDrive(0, 0);				
		
		Util.consoleLog("end: actual count=%d  active=%b", Math.abs(Devices.wheelEncoder.get()), isAutoActive());
	}
	
	/**
	 * Auto drive straight in set direction and power for specified encoder count. Stops
	 * with or without brakes on CAN bus drive system. Uses NavX yaw to drive straight.
	 * @param power Power applied, + is forward.
	 * @param encoderCounts Target encoder counts to move.
	 * @param enableBrakes True is brakes on, false is brakes off.
	 * @param usePid True is use PID to control movement, false is simple drive.
	 * @param useHeading True is measure steering yaw from last set navx target heading, false is measure yaw
	 * from direction robot is pointing when driving starts.
	 */
	private void autoDrive3(double power, int encoderCounts, boolean enableBrakes, boolean usePid, boolean useHeading)
	{
		double			angle, steeringGain = .05, elapsedTime = 0, lastPidCallTime = 0, pidCallTime = 0;
		double			kP = .002, kI = 0.0, kD = 0.0;
		
		SynchronousPID	pid = null;

		Util.consoleLog("pwr=%.2f  count=%d  brakes=%b  pid=%b  hdg=%b", power, encoderCounts, enableBrakes, usePid,
				useHeading);

		Devices.SetCANTalonBrakeMode(enableBrakes);

		Devices.wheelEncoder.reset();
		Devices.wheelEncoder2.reset();
		
		if (robot.isClone) Timer.delay(0.3);
		
		// If not measuring yaw from current heading, reset yaw based on current direction robot is facing.
		
		if (!useHeading)
		{
			Util.consoleLog("before reset=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
			
			//Devices.navx.resetYaw();
			
			Util.consoleLog("after reset1=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
			
			Devices.navx.resetYawWait(1, 500);
			
			Util.consoleLog("after reset2=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
		}
		
		// If using PID to control distance, configure the PID object.
		
		if (usePid)
		{
			pid = new SynchronousPID(kP, kI, kD);
			
			pid.setOutputRange(0, power);
			pid.setSetpoint(encoderCounts);
			lastPidCallTime = Timer.getFPGATimestamp();
		}
		
		// Drive until we  get there.
		
		while (isAutoActive() && Math.abs(Devices.wheelEncoder.get()) < encoderCounts) 
		{
			LCD.printLine(4, "wheel encoder=%d  winch encoder=%d", Devices.wheelEncoder.get(), Devices.winchEncoder.get());

			// Use PID to determine the power applied. Should reduce power as we get close
			// to the target encoder value.
			
			if (usePid)
			{
				pidCallTime = Timer.getFPGATimestamp();
				elapsedTime = pidCallTime - lastPidCallTime;
				
				pid.calculate(Math.abs(Devices.wheelEncoder.get()), elapsedTime);
				
				lastPidCallTime = pidCallTime;
				
				power = pid.get();
				
				Util.consoleLog("error=%d  power2=%.2f  time=%f", pid.getError(), power, elapsedTime);
			}

			// Angle is negative if robot veering left, positive if veering right when going forward.
			// It is opposite when going backward. Note that for this robot, + power means forward and
			// - power means backward.
			
			if (useHeading)
				angle = Devices.navx.getHeadingYaw();
			else
				angle = Devices.navx.getYaw();
			
			// Invert yaw angle for backwards.
	
			if (power < 0) angle = -angle;

			LCD.printLine(5, "angle=%.2f", angle);
			
			Util.consoleLog("angle=%.2f  hdg=%.2f", angle, Devices.navx.getHeading());
			
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
			
			// Done in this branch for testing 4-23-18.
			
			Devices.robotDrive.curvatureDrive(power, Util.clampValue(-angle * steeringGain, 1.0), false);
			
			Timer.delay(.010);
		}

		Devices.robotDrive.stopMotor();				
		
		Util.consoleLog("end: actual count=%d  active=%b", Math.abs(Devices.wheelEncoder.get()), isAutoActive());
	}
	
	/**
	 * Auto rotate left or right the specified angle. Left/right from robots forward looking view.
	 * @param power Speed of rotation, + is rotate right, - is rotate left. Uses NavX yaw to measure
	 * rotation.
	 * @param angle Angle to rotate, always +.
	 */
	private void autoRotate(double power, int angle)
	{
		int		error = 0;
		double	power2 = 0, pFactor, kP = .05, minPower = .30;

		// Min power is determined experimentally for each robot as the lowest power that
		// will rotate the robot. We don't want the pid reduction in power at the end of
		// the rotation to fall below this level and cause the rotation to stall before done.
		
		Util.consoleLog("pwr=%.2f  angle=%d", power, angle);
		
		// Try to prevent over rotation.
		Devices.SetCANTalonBrakeMode(true);

		Devices.navx.resetYaw();
		
		// Start rotation.
		//Devices.robotDrive.tankDrive(power, -power);
		
		angle = navxFix(angle);

		while (isAutoActive() && Math.abs((int) Devices.navx.getYaw()) < angle) 
		{
			// Quick and dirty PID control to reduce power as we approach target angle.
			error = angle - Math.abs((int) Devices.navx.getYaw());
			pFactor = error * kP;
			
			if (power > 0)
				power2 = Util.clampValue(power * pFactor, minPower, power);
			else
				power2 = Util.clampValue(power * pFactor, power, -minPower);
				
			Devices.robotDrive.tankDrive(power2, -power2);
			
			Util.consoleLog("angle=%.2f  error=%d  pfactor=%.2f  power2=%.2f  hdg=%.2f", Devices.navx.getYaw(), error, 
					pFactor, power2, Devices.navx.getHeading());
			
			Timer.delay(.010);
		} 

		Util.consoleLog("end angle1=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
		
		// Stop rotation.
		Devices.robotDrive.tankDrive(0, 0);

		Util.consoleLog("end angle2=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());

		// Wait for robot to stop moving.
		//while (isAutoActive() && Devices.navx.isRotating()) {Timer.delay(.010);}
		Util.consoleLog("moving=%b", Devices.navx.isRotating());
		
		Util.consoleLog("end angle3=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
	}
	
	private void autoRotate2(double power, double targetHeading)
	{
		double	error, pFactor, kP = .03, power2, kMinPower = .15;

		Util.consoleLog("pwr=%.2f  hdg=%.2f", power, targetHeading);
		
		// Try to prevent over rotation.
		Devices.SetCANTalonBrakeMode(true);

		Devices.navx.setTargetHeading(targetHeading);
		
		error = Devices.navx.getHeadingYaw();
		
		while (isAutoActive() && !Util.checkRange(error, 1.0)) 
		{
			pFactor = error * kP;
			
			power2 = Util.clampValue(power * pFactor, power) * -1;
			
			if (Util.checkRange(power2, kMinPower))
				if (power2 < 0)
					power2 = -kMinPower;
				else
					power2 = kMinPower;
			
			Devices.robotDrive.curvatureDrive(0, power2, true);
			
			Util.consoleLog("power2=%.2f  hdg=%.2f  yaw=%.2f en=%b auto=%b", power2, Devices.navx.getHeading(), error, robot.isEnabled(), robot.isAutonomous());
			
			Timer.delay(.010);
			
			error = Devices.navx.getHeadingYaw();
		} 

		Util.consoleLog("end  hdg=%.2f  yaw=%.2f", Devices.navx.getHeading(), Devices.navx.getHeadingYaw());
		
		// Stop rotation.
		Devices.robotDrive.stopMotor();	//.tankDrive(0, 0);

		Util.consoleLog("1  hdg=%.2f  yaw=%.2f", Devices.navx.getHeading(), Devices.navx.getHeadingYaw());

		// Wait for robot to stop moving.
		Util.consoleLog("moving=%b", Devices.navx.isRotating());
		//while (isAutoActive() && Devices.navx.isRotating()) {Timer.delay(.10);}
		Util.consoleLog("moving=%b", Devices.navx.isRotating());
		
		Util.consoleLog("2  hdg=%.2f  yaw=%.2f  ena=%b  auto=%b", Devices.navx.getHeading(), Devices.navx.getHeadingYaw(),
				robot.isEnabled(), robot.isAutonomous());
	}
	
	/**
	 * Auto rotate the specified target angle from where the robot is currently pointing or rotate
	 * to a target heading.
	 * @param power Max power for rotation. When rotating an angle, -power is rotate left, +power is rotate right.
	 * When rotating to specified heading, power is always +.
	 * @param target Target angle to rotate from robot current direction -180..+180, or target heading to rotate to
	 * from robot current heading. Target heading cannot be more than 180 degrees from current heading.
	 * @param usePid False for simple rotation, true to use PID controller to manage the rotation.
	 * @param useHeading False is target is an angle, true is target is a heading.
	 */
	private void autoRotate3(double power, double target, boolean usePid, boolean useHeading)
	{
		double	kP = .03, kI = 0.0, kD = 0.0, elapsedTime, lastPidCallTime = 0, pidCallTime = 0;
		
		SynchronousPID	pid = null;

		Util.consoleLog("pwr=%.2f  target=%.2f  pid=%b  hdg=%b", power, target, usePid, useHeading);
		
		// Try to prevent over rotation.
		Devices.SetCANTalonBrakeMode(true);

		// Reset yaw to current robot direction or target heading.
		
		if (useHeading) 
			Devices.navx.setTargetHeading(target);
		else
			Devices.navx.resetYawWait(1, 500);
		
		// Use PID to control power as we turn slowing as we approach target heading.
		
		if (usePid)
		{
			pid = new SynchronousPID(kP, kI, kD);
			
			pid.setOutputRange(0, power);
			pid.setSetpoint(target);
			lastPidCallTime = Timer.getFPGATimestamp();
			
			while (isAutoActive() && !pid.onTarget(1.0)) 
			{
				pidCallTime = Timer.getFPGATimestamp();
				elapsedTime = pidCallTime - lastPidCallTime;
				
				pid.calculate(Math.abs(Devices.wheelEncoder.get()), elapsedTime);
				
				lastPidCallTime = pidCallTime;
				
				power = pid.get();
				
				Devices.robotDrive.curvatureDrive(0, power, true);
				
				Util.consoleLog("power2=%.2f  hdg=%.2f  yaw=%.2f  time=%f  en=%b auto=%b", power, Devices.navx.getHeading(), 
						pid.getError(), elapsedTime, robot.isEnabled(), robot.isAutonomous());
				
				Timer.delay(.010);
			} 
		}
		else
		{
			while (isAutoActive() && Math.abs((int) Devices.navx.getYaw()) < target) 
			{
				Devices.robotDrive.curvatureDrive(0, power, true);
				
				Util.consoleLog("angle=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
				
				Timer.delay(.010);
			} 			
		}
		
		Util.consoleLog("end loop  hdg=%.2f  yaw=%.2f", Devices.navx.getHeading(), Devices.navx.getHeadingYaw());
		
		// Stop rotation.
		Devices.robotDrive.stopMotor();	//.tankDrive(0, 0);

		Util.consoleLog("after stop  hdg=%.2f  yaw=%.2f", Devices.navx.getHeading(), Devices.navx.getHeadingYaw());

		// Wait for robot to stop moving.
		Util.consoleLog("moving=%b", Devices.navx.isRotating());
		//while (isAutoActive() && Devices.navx.isRotating()) {Timer.delay(.10);}
		//Util.consoleLog("moving=%b", Devices.navx.isRotating());
		
		Util.consoleLog("2  hdg=%.2f  yaw=%.2f  ena=%b  auto=%b", Devices.navx.getHeading(), Devices.navx.getHeadingYaw(),
				robot.isEnabled(), robot.isAutonomous());
	}

	/**
	 * Drive in S curve, curve one direction, drive straight, curve back to starting heading.
	 * @param power Speed to drive, + is forward.
	 * @param curve Rate of rotation (-1.0 <-> 1.0), + is right or clockwise.
	 * @param targetAngle Angle to turn, always +.
	 * @param straightEncoderCounts Counts to travel on straight leg, always +.
	 */
	private void autoSCurve(double power, double curve, int targetAngle, int straightEncoderCounts)
	{
		
		Util.consoleLog("pwr=%.2f  curve=%.2f  angle=%d  counts=%d", power, curve, targetAngle, straightEncoderCounts);
		
		targetAngle = navxFix(targetAngle);
		
		// We start out driving in a curve until we have turned the desired angle.
		// Then we drive straight the desired distance then curve back to starting
		// heading. Curve is + for right, - for left.
		
		Devices.robotDrive.curvatureDrive(power, curve, false);
		
		while (isAutoActive() && Math.abs((int) Devices.navx.getYaw()) < targetAngle) 
		{
			Timer.delay(.010);
			LCD.printLine(6, "angle=%.2f", Devices.navx.getYaw());
			Util.consoleLog("angle=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
		}

		Util.consoleLog("end angle=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());

		autoDrive(power, straightEncoderCounts, false);
		
		Devices.navx.resetYaw();
		
		Devices.SetCANTalonBrakeMode(true);

		// Reduce target angle to stop over rotation.
		targetAngle -= 10;
		
		// Reduce power so don't slam the switch too hard.
		Devices.robotDrive.curvatureDrive(power * 0.7, -curve, false);
		
		while (isAutoActive() && Math.abs((int) Devices.navx.getYaw()) < targetAngle && (Devices.ds.getMatchType() != MatchType.None ? Devices.ds.getMatchTime() > 5 : true)) 
		{
			Timer.delay(.020);
			LCD.printLine(6, "angle=%.2f", Devices.navx.getYaw());
			Util.consoleLog("angle=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
		}

		Util.consoleLog("end angle=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());

		Devices.robotDrive.tankDrive(0, 0);
	}
	
	private enum PlateStates
	{
		UNDEFINED,
		LLL,
		RRR,
		LRL,
		RLR;
 	}
	
	// Correction for clone navx which reads 15 degrees to low. This routine adjusts
	// the target angle for the error.
	
	private int navxFix(int targetAngle)
	{
		if (robot.isComp)
			return targetAngle;
		else
		{
			Util.consoleLog("ta=%d  fa=%d", targetAngle, Math.round(targetAngle * .8333f));
			return Math.round(targetAngle * .8333f);
		}
	}
}