
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
		
		// Update the robot heading indicator on the DS.

		SmartDashboard.putNumber("Gyro", Devices.navx.getHeadingInt());
		
		Util.consoleLog("final heading=%.2f", Devices.navx.getHeading());
		
		Util.consoleLog("end");
	}

	// Start from left or right and just drive across the line.
	
	private void startOutsideNoScore()
	{
		Util.consoleLog();
		
		autoDrive3(.50, 2490, true, true, true, true);	// 1606
	}

	// Start from center (offset right). Drive forward to break the line and stop.
 
	private void startCenterNoScore()
	{
		Util.consoleLog();
		
		autoDrive3(.50, 1970, true, true, true, true);
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
		
		autoDrive3(.40, 925, true, true, true, true);		// 596
		
		switch (plateState)
		{
			case UNDEFINED:
				startCenterNoScore();
				return;
				
			case LLL: case LRL:
				autoRotate3(.30, 270, true, true);
				autoDrive3(.60, 928, true, true, true, true);		// 663
				autoRotate3(.30, 0, true, true);
				autoDrive3(.60, 880, true, true, true, true);		// 567
				break;
				
			case RRR: case RLR:
				autoRotate3(.30, 90, true, true);
				autoDrive3(.60, 900, true, true, true, true);		// 857
				autoRotate3(.30, 0, true, true);
				autoDrive3(.60, 880, true, true, true, true);		// 567
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
		
		autoDrive3(.40, 100, true, true, true, true);		// 596
		
		switch (plateState)
		{
			case UNDEFINED:
				startCenterNoScore();
				return;
				
			case LLL: case LRL:
				autoRotate3(.50, 334, true, true);				// 26 -.50
				autoDrive3(.60, 2100, true, true, true, true);		// 663
				break;
				
			case RRR: case RLR:
				autoRotate3(.50, 18, true, true);
				autoDrive3(.60, 1900, true, true, true, true);	// 857
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
				//autoSCurve(.50, -.3, 30, 900);
				autoSCurve2(.50, .30, 330, 900);

				break;
				
			case RRR: case RLR:
				//autoSCurve(.50, .3, 30, 950);
				autoSCurve2(.50, .30, 30, 950);
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
					autoDrive3(.50, 2500, true, true, true, true);	// 4600/2967
					
					// Drop the lift.
					lift.setHeight(-1);
					return;
					
				case LRL: case LLL:		// Switch available.
					autoDrive3(.50, 3180, true, true, true, true);	// 2051
					autoRotate3(.50, 90, true, true);
					autoDrive3(.30, 320, true, true, true, true);		// 206
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
					autoDrive3(.50, 2500, true, true, true, true);	// 4600/2967
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
					autoDrive3(.50, 3180, true, true, true, true);	// 2051
					autoRotate3(.50, 270, true, true);
					autoDrive3(.30, 320, true, true, true, true);		// 206s
					break;
			}
		}
		
		// Dump cube.
		
		grabber.spit(spitPower);
		
	}
	
	/**
	 * Auto drive straight in set direction and power for specified encoder count. Stops
	 * with or without brakes on CAN bus drive system. Uses NavX yaw from current direction to drive straight.
	 * @param power Speed, + is forward.
	 * @param encoderCounts encoder counts to travel, always +.
	 * @param enableBrakes True to enable brakes.
	 */
	private void autoDrive(double power, int encoderCounts, boolean enableBrakes)
	{
		int		yaw;
		double	gain = .05;

		// Min power is determined experimentally for each robot as the lowest power that
		// will move the robot. We don't want the pid reduction in power at the end of
		// the drive to fall below this level and cause the drive to stall before done.
		
		Util.consoleLog("pwr=%.2f  count=%d  brakes=%b", power, encoderCounts, enableBrakes);

		Devices.SetCANTalonBrakeMode(enableBrakes);

		Devices.wheelEncoder.reset();
		Devices.wheelEncoder2.reset();
		
		Util.consoleLog("before reset=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
			
		Devices.navx.resetYawWait(1, 500);
		
		Util.consoleLog("after reset2=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
		
		while (isAutoActive() && Math.abs(Devices.wheelEncoder.get()) < encoderCounts) 
		{
			LCD.printLine(4, "wheel encoder=%d  winch encoder=%d", Devices.wheelEncoder.get(), Devices.winchEncoder.get());

			// Yaw is negative if robot veering left, positive if veering right when going forward.
			// It is opposite when going backward. Note that for this robot, + power means forward and
			// - power means backward.
			
			yaw = (int) Devices.navx.getYaw();
			
			// Invert yaw angle for backwards.
	
			//if (power < 0) yaw = -yaw;

			LCD.printLine(5, "yaw=%d", yaw);
			
			Util.consoleLog("yaw=%d  hdg=%.2f", yaw, Devices.navx.getHeading());
			
			// Note we invert sign on the yaw angle because we want the robot to turn in the opposite
			// direction than it is currently going to correct it. So a + yaw angle says robot is veering
			// right so we set the turn value to - because - is a turn left which corrects our right
			// drift.
			
			Devices.robotDrive.curvatureDrive(power, Util.clampValue(-yaw * gain, 1.0), false);
			
			Timer.delay(.010);
		}

		Devices.robotDrive.tankDrive(0, 0);				
		
		Util.consoleLog("end: actual count=%d", Math.abs(Devices.wheelEncoder.get()));
	}
	
	/**
	 * Auto drive straight in set direction and power for specified encoder count. Stops
	 * with or without brakes on CAN bus drive system. Uses NavX yaw from current target heading
	 * (as set with Navx.setTargetHeading()) to actual heading to drive straight.
	 * @param power Speed, + is forward.
	 * @param encoderCounts encoder counts to travel, always +.
	 * @param enableBrakes True to enable brakes.
	 */
	private void autoDrive2(double power, int encoderCounts, boolean enableBrakes)
	{
		double	yaw, gain = .05;
		
		Util.consoleLog("pwr=%.2f  count=%d  brakes=%b", power, encoderCounts, enableBrakes);

		Devices.SetCANTalonBrakeMode(enableBrakes);

		Devices.wheelEncoder.reset();
		Devices.wheelEncoder2.reset();
		
		while (isAutoActive() && Math.abs(Devices.wheelEncoder.get()) < encoderCounts) 
		{
			LCD.printLine(4, "wheel encoder=%d  winch encoder=%d", Devices.wheelEncoder.get(), Devices.winchEncoder.get());

			// Yaw angle is negative if robot veering left, positive if veering right when going forward.
			// It is opposite when going backward. Note that for this robot, + power means forward and
			// - power means backward.
			
			yaw = Devices.navx.getHeadingYaw();
			
			// Invert yaw angle for backwards.a
	
			//if (power < 0) yaw = -yaw;

			LCD.printLine(5, "yaw=%.2f", yaw);
			
			Util.consoleLog("yaw=%.2f  hdg=%.2f", yaw, Devices.navx.getHeading());
			
			// Note we invert sign on the angle because we want the robot to turn in the opposite
			// direction than it is currently going to correct it. So a + angle says robot is veering
			// right so we set the turn value to - because - is a turn left which corrects our right
			// drift.
			
			Devices.robotDrive.curvatureDrive(power, Util.clampValue(-yaw * gain, 1.0), false);
			
			Timer.delay(.010);
		}

		Devices.robotDrive.stopMotor();		
		
		Util.consoleLog("end: actual count=%d  active=%b", Math.abs(Devices.wheelEncoder.get()), isAutoActive());
	}
	
	/**
	 * Auto drive straight in set direction and power for specified encoder count. Stops
	 * with or without brakes on CAN bus drive system. Uses NavX yaw to drive straight.
	 * @param power Power applied, + is forward.
	 * @param encoderCounts Target encoder counts to move, always +.
	 * @param stop True stops motors at end of curve, false leaves power on to flow into next move.
	 * @param enableBrakes True is brakes on, false is brakes off.
	 * @param usePid True is use PID to control movement, false is simple drive.
	 * @param useHeading True is measure steering yaw from last set navx target heading, false is measure yaw
	 * from direction robot is pointing when driving starts.
	 * 
	 * Note: This routine is designed for tank drive and the P,I,D values will likely need adjusting for each
	 * new drive base as gear ratios and wheel configuration may require different values to stop smoothly
	 * and accurately.
	 */
	private void autoDrive3(double power, int encoderCounts, boolean stop, boolean enableBrakes, boolean usePid, 
						    boolean useHeading)
	{
		double			yaw, steeringGain = .07, elapsedTime = 0, lastPidCallTime = 0, pidCallTime = 0;
		double			kP = .002, kI = 0.001, kD = 0.001;
		
		SynchronousPID	pid = null;

		Util.consoleLog("pwr=%.2f  count=%d  stop=%b  brakes=%b  pid=%b  hdg=%b", power, encoderCounts, stop, enableBrakes, 
				usePid,	useHeading);

		Util.checkRange(power, 1.0);
		
		if (encoderCounts <= 0) throw new IllegalArgumentException("Encoder counts < 1");
		
		Devices.SetCANTalonBrakeMode(enableBrakes);

		Devices.wheelEncoder.reset();
		Devices.wheelEncoder2.reset();
		
		if (robot.isClone) Timer.delay(0.3);
		
		// If not measuring yaw from current heading, reset yaw based on current direction robot is facing.
		
		if (!useHeading)
		{
			Util.consoleLog("before reset=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
			
			Devices.navx.resetYawWait(1, 500);
			
			Util.consoleLog("after reset2=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
		}
		
		// If using PID to control distance, configure the PID object.
		
		if (usePid)
		{
			pid = new SynchronousPID(kP, kI, kD);
			
			if (power < 0)
			{
				pid.setSetpoint(-encoderCounts);
				pid.setOutputRange(power, 0);
			}
			else
			{
				pid.setSetpoint(encoderCounts);
				pid.setOutputRange(0, power);
			}

			lastPidCallTime = Timer.getFPGATimestamp();
		}
		
		// Drive until we get there.
		
		while (isAutoActive() && Math.abs(Devices.wheelEncoder.get()) < encoderCounts) 
		{
			LCD.printLine(4, "wheel encoder=%d  winch encoder=%d", Devices.wheelEncoder.get(), Devices.winchEncoder.get());

			// Use PID to determine the power applied. Should reduce power as we get close
			// to the target encoder value.
			
			if (usePid)
			{
				pidCallTime = Timer.getFPGATimestamp();
				elapsedTime = pidCallTime - lastPidCallTime;
				
				pid.calculate(Devices.wheelEncoder.get(), elapsedTime);
				
				lastPidCallTime = pidCallTime;
				
				power = pid.get();
				
				Util.consoleLog("error=%.2f  power2=%.2f  time=%f", pid.getError(), power, elapsedTime);
			}

			// Yaw angle is negative if robot veering left, positive if veering right when going forward.
			
			if (useHeading)
				yaw = Devices.navx.getHeadingYaw();
			else
				yaw = Devices.navx.getYaw();
			
			LCD.printLine(5, "yaw=%.2f", yaw);
			
			Util.consoleLog("yaw=%.2f  hdg=%.2f  rot=%.2f", yaw, Devices.navx.getHeading(), -yaw * steeringGain);
			
			// Note we invert sign on the angle because we want the robot to turn in the opposite
			// direction than it is currently going to correct it. So a + angle says robot is veering
			// right so we set the turn value to - because - is a turn left which corrects our right
			// drift. SteeringGain controls how aggressively we turn to stay on course.
			
			Devices.robotDrive.curvatureDrive(power, Util.clampValue(-yaw * steeringGain, 1.0), false);
			
			Timer.delay(.010);
		}

		if (stop) Devices.robotDrive.stopMotor();				
		
		Util.consoleLog("end: actual count=%d  error=%.3f  ena=%b  isa=%b", Math.abs(Devices.wheelEncoder.get()), 
				(double) Math.abs(Devices.wheelEncoder.get()) / encoderCounts, robot.isEnabled(), robot.isAutonomous());
	}
	
	/**
	 * Auto rotate left or right the specified angle. Left/right from robots forward looking view.
	 * @param power Speed of rotation, + is rotate right, - is rotate left. Uses NavX yaw to measure
	 * rotation. This is the original autoRotate function.
	 * @param angle Angle to rotate, always +.
	 */
	private void autoRotate(double power, int angle)
	{
		Util.consoleLog("pwr=%.2f  angle=%d", power, angle);
		
		// Try to prevent over rotation.
		Devices.SetCANTalonBrakeMode(true);

		Devices.navx.resetYaw();
		
		while (isAutoActive() && Math.abs((int) Devices.navx.getYaw()) < angle) 
		{
			Devices.robotDrive.tankDrive(power, -power);
			
			Util.consoleLog("yaw=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
			
			Timer.delay(.010);
		} 

		Util.consoleLog("endloop yaw=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
		
		// Stop rotation.
		Devices.robotDrive.stopMotor();

		Util.consoleLog("end yaw=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
	}
	
	/**
	 * Auto rotate to new heading. 
	 * @param power Power to rotate at. Always +.
	 * @param targetHeading New heading in degrees. Heading 0 always points down the field
	 * and is set in autonomous and maintained from there. Left or right rotate determined
	 * by the initial yaw between current and target headings.
	 */
	private void autoRotate2(double power, double targetHeading)
	{
		double	yaw;

		Util.consoleLog("pwr=%.2f  hdg=%.2f", power, targetHeading); 
		
		// Try to prevent over rotation.
		Devices.SetCANTalonBrakeMode(true);

		Devices.navx.setTargetHeading(targetHeading);
		
		yaw = Devices.navx.getHeadingYaw();
		
		if (yaw < 0) power = power * -1;
		
		// 1 degree target tolerance.
		
		while (isAutoActive() && !Util.checkRange(yaw, 1.0)) 
		{
			Devices.robotDrive.curvatureDrive(0, power, true);
			
			Util.consoleLog("hdg=%.2f  yaw=%.2f  ena=%b", Devices.navx.getHeading(), yaw, robot.isEnabled());
			
			Timer.delay(.010);
			
			yaw = Devices.navx.getHeadingYaw();
		} 

		Util.consoleLog("endloop  hdg=%.2f  yaw=%.2f", Devices.navx.getHeading(), yaw);
		
		// Stop rotation.
		Devices.robotDrive.stopMotor();

		Util.consoleLog("end hdg=%.2f  yaw=%.2f  ena=%b", Devices.navx.getHeading(), Devices.navx.getHeadingYaw(),
				robot.isEnabled());
	}
	
	/**
	 * Auto rotate the specified target angle from where the robot is currently pointing or rotate
	 * to a target heading.
	 * @param power Max power for rotation. Power is always +.
	 * @param target Target angle (-left, +right) to rotate from robot current direction -180..+180, or 
	 * target heading (0..359) to rotate to from robot current heading. Target heading is always + and cannot be more 
	 * than 180 degrees away from current heading.
	 * @param usePid False for simple rotation, true use PID controller to manage the rotation slowing rotation as
	 * target is reached.
	 * @param useHeading False target is an angle, true target is a heading.
	 * 
	 * Note: This routine is designed for tank drive and the P,I,D values will likely need adjusting for each
	 * new drive base as gear ratios and wheel configuration may require different values to turn smoothly
	 * and accurately.
	 */
	private void autoRotate3(double power, double target, boolean usePid, boolean useHeading)
	{
		double	kP = .02, kI = 0.003, kD = 0.001, kTolerance = 1.0;
		double	elapsedTime, lastPidCallTime = 0, pidCallTime = 0, yaw = 0;
		
		SynchronousPID	pid = null;

		Util.consoleLog("pwr=%.2f  target=%.2f  pid=%b  hdg=%b", power, target, usePid, useHeading);
		
		if (power <= 0) throw new IllegalArgumentException("power");
		
		// Try to prevent over rotation.
		Devices.SetCANTalonBrakeMode(true);

		// Reset yaw to current robot direction or target heading.
		
		if (useHeading) 
		{
			Util.checkRange(target, 0, 359, "target");
			
			Devices.navx.setTargetHeading(target);
		}
		else
		{
			Util.checkRange(target, 180, "target");
			
			Devices.navx.resetYawWait(1, 500);
		}
		
		if (usePid)
		{
			// Use PID to control power as we turn slowing as we approach target heading.
			
			pid = new SynchronousPID(kP, kI, kD);
			
			pid.setOutputRange(-power , power);
			
			if (useHeading)
				pid.setSetpoint(0);			// We are trying to get the yaw to zero.
			else
				pid.setSetpoint(target);	// We are trying to get to the target yaw.
			
			// The PID class needs delta time between calls to calculate the I term.
			
			lastPidCallTime = Timer.getFPGATimestamp();
			
			while (isAutoActive() && !pid.onTarget(kTolerance)) 
			{
				pidCallTime = Timer.getFPGATimestamp();
				elapsedTime = pidCallTime - lastPidCallTime;
				
				if (useHeading)
					yaw = Devices.navx.getHeadingYaw();
				else
					yaw = Devices.navx.getYaw();

				// Our target is zero yaw so we determine the difference between
				// current yaw and target and perform the PID calculation which
				// results in the speed of turn, reducing power as the difference
				// approaches zero. So our turn should slow and not overshoot. If
				// it does, the PID controller will reverse power and turn it back.
				
				pid.calculate(yaw, elapsedTime);
				
				lastPidCallTime = pidCallTime;
				
				power = pid.get();
				
				// When quick turn is true, first parameter is not used, power is fed to the
				// rate of turn parameter. PID controller takes care of the sign, that 
				// is the left/right direction of the turn.
				
				Devices.robotDrive.curvatureDrive(0, power, true);
				
				Util.consoleLog("power=%.2f  hdg=%.2f  yaw=%.2f  err=%.2f  time=%f  ena=%b", power, Devices.navx.getHeading(), 
						yaw, pid.getError(), elapsedTime, robot.isEnabled());
				
				Timer.delay(.010);
			} 
		}
		else
		{
			// Simple turn, full power until target reached.
			
			if (useHeading)
			{
				yaw = Devices.navx.getHeadingYaw();

				if (yaw > 0) power = power * -1;
				
				while (isAutoActive() && !Util.checkRange(yaw, 1.0)) 
				{
					Devices.robotDrive.curvatureDrive(0, power, true);
					
					Util.consoleLog("yaw=%.2f  hdg=%.2f", yaw, Devices.navx.getHeading());
					
					Timer.delay(.010);
					
					yaw = Devices.navx.getHeadingYaw();
				}
			}
			else
			{
				yaw = Devices.navx.getYaw();
			
				if (target < 0) power = power * -1;
			
				while (isAutoActive() && Math.abs(yaw) < Math.abs(target)) 
				{
					Devices.robotDrive.curvatureDrive(0, power, true);
					
					Util.consoleLog("yaw=%.2f  hdg=%.2f", yaw, Devices.navx.getHeading());
					
					Timer.delay(.010);
					
					yaw = Devices.navx.getYaw();
				}
			}
		}
		
		Util.consoleLog("end loop  hdg=%.2f  yaw=%.2f", Devices.navx.getHeading(), yaw);
		
		// Stop rotation.
		Devices.robotDrive.stopMotor();

		Util.consoleLog("after stop  hdg=%.2f  yaw=%.2f", Devices.navx.getHeading(), yaw);

		// Wait for robot to stop moving.
		Util.consoleLog("moving=%b", Devices.navx.isRotating());
		//while (isAutoActive() && Devices.navx.isRotating()) {Timer.delay(.10);}
		//Util.consoleLog("moving=%b", Devices.navx.isRotating());
		
		Util.consoleLog("2  hdg=%.2f  yaw=%.2f  ena=%b", Devices.navx.getHeading(), yaw, robot.isEnabled());
	}
	
	/**
	 * Automatically drive in a curve.
	 * @param power Speed to drive, + is forward.
	 * @param curve Speed of rotation 0..1.0, always +.
	 * @param target Target angle to turn. If not using heading, this is 0..180, - left, + right. If using heading
	 * this is the target heading 0..359.
	 * @param stop True stops motors at end of curve, false leaves power on to flow into next move.
	 * @param useBrakes True turns on brakes for end of curve.
	 * @param usePid True uses PID controller to manage the curve slowing rotation as target is reached. False
	 * uses the fixed curve value for whole rotation.
	 * @param useHeading True target is a heading, false target is an angle from current direction.
	 */
	private void autoCurve(double power, double curve, double target, boolean stop, boolean useBrakes, boolean usePid, 
						   boolean useHeading)
	{
		double	kP = .05, kI = 0.001, kD = 0.001, kTolerance= 1.0;
		double	elapsedTime, lastPidCallTime = 0, pidCallTime = 0, yaw = 0;
		
		SynchronousPID	pid = null;

		Util.consoleLog("pwr=%.2f  curve=%.2f  target=%.2f  stop=%b  brakes=%b  pid=%b  hdg=%b", power, curve, target, 
						stop, useBrakes, usePid, useHeading);
		
		Util.checkRange(power, 1.0, "power");
		
		Util.checkRange(curve, 0, 1.0, "curve");
		
		Devices.SetCANTalonBrakeMode(useBrakes);

		// Reset yaw to current robot direction or target heading.
		
		if (useHeading) 
		{
			Util.checkRange(target, 0, 359.999, "target");
			
			Devices.navx.setTargetHeading(target);
		}
		else
		{
			Util.checkRange(target, 180, "target");
			
			Devices.navx.resetYawWait(1, 500);
		}
		
		if (usePid)
		{
			// Use PID to control power as we turn slowing as we approach target heading.
			
			pid = new SynchronousPID(kP, kI, kD);
			
			pid.setOutputRange(-Math.abs(curve) , Math.abs(curve));
			
			if (useHeading)
				pid.setSetpoint(0);			// We are trying to get the yaw to zero.
			else
				pid.setSetpoint(target);	// We are trying to get to the target yaw.
			
			// The PID class needs delta time between calls to calculate the I term.
			
			lastPidCallTime = Timer.getFPGATimestamp();
			
			while (isAutoActive() && !pid.onTarget(kTolerance)) 
			{
				pidCallTime = Timer.getFPGATimestamp();
				elapsedTime = pidCallTime - lastPidCallTime;
				
				if (useHeading)
					yaw = Devices.navx.getHeadingYaw();
				else
					yaw = Devices.navx.getYaw();

				// Our target is zero yaw so we determine the difference between
				// current yaw and target and perform the PID calculation which
				// results in the speed (curve) of turn, reducing curve as the difference
				// approaches zero. So our turn should slow and not overshoot. If
				// it does, the PID controller will reverse curve and turn it back.
				
				pid.calculate(yaw, elapsedTime);
				
				lastPidCallTime = pidCallTime;
				
				curve = pid.get();
				
				// When quick turn is false, power is constant, curve is fed to the
				// rate of turn parameter. PID controller takes care of the sign, that 
				// is the left/right direction of the turn.
				
				Devices.robotDrive.curvatureDrive(power, curve, false);
				
				Util.consoleLog("power=%.2f  hdg=%.2f  yaw=%.2f  curve=%.2f  err=%.2f  time=%f  ena=%b", power, Devices.navx.getHeading(), 
						yaw, curve, pid.getError(), elapsedTime, robot.isEnabled());
				
				Timer.delay(.010);
			} 
		}
		else
		{
			// Simple turn, full curve until target reached.
			
			if (useHeading)
			{
				yaw = Devices.navx.getHeadingYaw();

				if (yaw > 0) curve = curve * -1;
				
				while (isAutoActive() && !Util.checkRange(yaw, 1.0)) 
				{
					Devices.robotDrive.curvatureDrive(power, curve, false);
					
					Util.consoleLog("yaw=%.2f  hdg=%.2f  curve=%.2f", yaw, Devices.navx.getHeading(), curve);
					
					Timer.delay(.010);
					
					yaw = Devices.navx.getHeadingYaw();
				}
			}
			else
			{
				yaw = Devices.navx.getYaw();
			
				if (target < 0) curve = curve * -1;
						
				while (isAutoActive() && Math.abs(yaw) < Math.abs(target)) 
				{
					Devices.robotDrive.curvatureDrive(power, curve, false);
					
					Util.consoleLog("yaw=%.2f  hdg=%.2f  curve=%.2f", yaw, Devices.navx.getHeading(), curve);
					
					Timer.delay(.010);
					
					yaw = Devices.navx.getYaw();
				}
			}
		}
		
		Util.consoleLog("end loop  hdg=%.2f  yaw=%.2f", Devices.navx.getHeading(), yaw);
		
		// Stop motors.
		if (stop) Devices.robotDrive.stopMotor();

		Util.consoleLog("after stop  hdg=%.2f  yaw=%.2f", Devices.navx.getHeading(), yaw);

		// Wait for robot to stop moving.
		Util.consoleLog("moving=%b", Devices.navx.isRotating());
		//while (isAutoActive() && Devices.navx.isRotating()) {Timer.delay(.10);}
		//Util.consoleLog("moving=%b", Devices.navx.isRotating());
		
		Util.consoleLog("end hdg=%.2f  yaw=%.2f  ena=%b", Devices.navx.getHeading(), yaw, robot.isEnabled());
	}

	/**
	 * Drive in S curve, curve one direction, drive straight, curve back to starting heading.
	 * @param power Speed to drive, + is forward.
	 * @param curve Rate of rotation (-1.0..1.0), + is right or clockwise.
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
	
	/**
	 * Drive in S curve, curve one direction, drive straight, curve back to starting heading.
	 * @param power Speed to drive + is forward.
	 * @param curve Rate of rotation (0..1.0), always +. PID controller will turn left/right
	 * as needed to reach target heading.
	 * @param target Heading to turn to 0..359.
	 * @param straightEncoderCounts Length of straight led between curves.
	 */
	private void autoSCurve2(double power, double curve, double target, int straightEncoderCounts)
	{
		double	saveHeading = Devices.navx.getHeading();
		
		Util.consoleLog("pwr=%.2f  curve=%.2f  target=%.2f  counts=%d", power, curve, target, straightEncoderCounts);
		
		Devices.SetCANTalonRampRate(0.5);	

		// We start out driving in a curve until we have turned to the desired heading.
		// Then we drive straight the desired distance then curve back to starting
		// heading. PID controller in autoCurve will determine the sign of curve.
		
		autoCurve(-power, curve, target, false, false, true, true);
		
		autoDrive3(-power, straightEncoderCounts, false, false, false, true);
		
		autoCurve(-power, curve, saveHeading, true, true, true, true);
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