
package Team4450.Robot11;

import java.io.File;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
//import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class Autonomous
{
	private final Robot		robot;
	private final int		program = (int) SmartDashboard.getNumber("AutoProgramSelect",0);
	private final GearBox	gearBox;
	
	Autonomous(Robot robot)
	{
		Util.consoleLog();
		
		this.robot = robot;
		
		gearBox = new GearBox(robot);
	}

	public void dispose()
	{
		Util.consoleLog();
		
		if (gearBox != null) gearBox.dispose();
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

		// Initialize encoders.
		Devices.wheelEncoder.reset();
		Devices.rightEncoder.reset();
		Devices.leftEncoder.reset();
		
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
				pathFinderTest();
				break;
				
			case 2:
				velocityTest1();
				break;
				
			case 3:
				VelocityTest2();
				break;
		}
		
		// Update the robot heading indicator on the DS.

		SmartDashboard.putNumber("Gyro", Devices.navx.getHeadingInt());
		
		Util.consoleLog("final heading=%.2f  R=%.2f", Devices.navx.getHeading(), Devices.navx.getHeadingR());
		
		Util.consoleLog("end");
	}

	private void pathFinderTest()
	{
		File	file = new File("/home/lvuser/latest.trajectory");
		
		Util.consoleLog();
		
		// Rotate to starting heading of 45 degrees, 315 in Pathfinder orientation.
		autoRotate(.50, 45, true, true);
		
		Pathfinder.setTrace(true);
		
		// 3 Waypoints. Distances in meters. Working back from end point.
		// Notes: PF sees headings as starting a zero and proceeding to 1
		// degree, 2 degrees, and so on, left (that is counter clockwise)
		// coming around to 359 just right (clockwise) of zero. This is
		// opposite the way we do it in our NavX headings. NavX class has
		// a method to return the heading in this scheme.
		//
		// Next, angles are specified as negative to turn right (clockwise)
		// and positive to turn left (counter clockwise).
		Waypoint[] points = new Waypoint[] 
		{
//		    new Waypoint(-2, 0, Pathfinder.d2r(-45)),      	// Waypoint @ x=-4, y=-1, exit angle=-45 degrees
//		    new Waypoint(-1, -1, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
//		    new Waypoint(0, 0, 0)                           // Waypoint @ x=0, y=0,   exit angle=0 radians
				
			    new Waypoint(0, 0, Pathfinder.d2r(-45)),      // Waypoint @ x=0, y=0, exit angle=-45 degrees
			    new Waypoint(2, -1, 0),                       // Waypoint @ x=2, y=-1, exit angle=0 radians
			    new Waypoint(4, 0, 0)                         // Waypoint @ x=4, y=0,   exit angle=0 radians
				
//			    new Waypoint(0, 0, 0),      				// Waypoint @ x=0, y=0, exit angle=0 degrees
//			    new Waypoint(1, 0, 0),                      // Waypoint @ x=2, y=0, exit angle=0 radians
//			    new Waypoint(3, -1, Pathfinder.d2r(-45))                     // Waypoint @ x=2, y=-2 exit angle=0 radians
			    //new Waypoint(0, -2, 0)

//				new Waypoint(0, 0, 0),      				// Waypoint @ x=0, y=0, exit angle=0 degrees
//			    new Waypoint(1, 0,  0),                      // Waypoint @ x=2, y=0, exit angle=0 radians
//			    new Waypoint(2, -1, 0),                     // Waypoint @ x=2, y=-2 exit angle=0 radians
//			    new Waypoint(-1, -2, 0),
//			    new Waypoint(0, -2, 0)
		};

		// Create the Trajectory Configuration
		//
		// Arguments:
		// Fit Method:          HERMITE_CUBIC or HERMITE_QUINTIC
		// Sample Count:        SAMPLES_HIGH (100 000)
		//                      SAMPLES_LOW  (10 000)
		//                      SAMPLES_FAST (1 000)
		// Time Step:           0.05 Seconds
		// Max Velocity:        1.7 m/s @ 100% power
		// Max Acceleration:    2.0 m/s/s
		// Max Jerk:            60.0 m/s/s/s
		
		double max_velocity = 0.5;	//1.7;
		double max_acceleration = 0.5;
		double max_jerk = 60.0;
		double time_step = .05;
		
		Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, 
														 Trajectory.Config.SAMPLES_HIGH, 
														 time_step, 
														 max_velocity, 
														 max_acceleration, 
														 max_jerk);

		// Generate the trajectory
		Trajectory trajectory = Pathfinder.generate(points, config, file);
		
		//Trajectory trajectory = Pathfinder.readFromCSV(file);
		
		// The distance between the left and right sides of the wheel base is 28 inches or 0.7112 meters.
		// Wheel diameter is 5.8 inches or .14732 meters.
		double wheelbase_width = Util.inchesToMeters(28);
		double wheel_diameter = Util.inchesToMeters(5.8);

		// Create the Modifier Object
		TankModifier modifier = new TankModifier(trajectory);

		// Generate the Left and Right trajectories using the original trajectory
		// as the center.
		modifier.modify(wheelbase_width);

		Trajectory leftTrajectory = modifier.getLeftTrajectory();
		Trajectory rightTrajectory = modifier.getRightTrajectory();
		
		// Create encoder follower for each encoder.
		EncoderFollower left = new EncoderFollower(leftTrajectory, "left");
		EncoderFollower right = new EncoderFollower(rightTrajectory, "right");
		
		// Encoder Position is the current, cumulative position of your encoder. 
		// 4096 is the amount of encoder ticks per full revolution for SRX magnetic encoder.
		// Wheel Diameter is the diameter of your wheels (or pulley for a track system) in meters.
		left.configureEncoder(Devices.leftEncoder.get(), 4096, wheel_diameter);
		right.configureEncoder(Devices.rightEncoder.get(), 4096, wheel_diameter);
		
		// The first argument is the proportional gain. Usually this will be quite high.
		// The second argument is the integral gain. This is unused for motion profiling.
		// The third argument is the derivative gain. Tweak this if you are unhappy with the tracking of the trajectory.
		// The fourth argument is the velocity ratio. This is 1 over the maximum velocity @ 100% power you provided in the 
		// trajectory configuration (it translates m/s to a -1 to 1 scale that your motors can read).
		// The fifth argument is your acceleration gain. Tweak this if you want to get to a higher or lower speed quicker.
		left.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0);	
		right.configurePIDVA(1.0, 0.0, 0.0, 1 / max_velocity, 0);	
		
		while (isAutoActive() && !left.isFinished())
		{
			double lPower = left.calculate(Devices.leftEncoder.get());
			double rPower = right.calculate(Devices.rightEncoder.get());

			double gyro_heading = Devices.navx.getHeadingR();			// degrees
			double desired_heading = Pathfinder.r2d(left.getHeading()); // Should also be in degrees

			double angleDifference = Pathfinder.boundHalfDegrees(desired_heading - gyro_heading);
			//double turn = 0.8 * (-1.0/80.0) * angleDifference;
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
		
		if (left.isFinished()) Util.consoleLog("reached end of trajectory");
	}
	
	private void velocityTest1()
	{
		double		kP = .10, kI = 0.0, kD = 0.0, kF = 0.10, elapsedTime = 0, power;
		
		SynchronousPID	pidController = null;

		Util.consoleLog();
		
		pidController = new SynchronousPID(kP, kI, kD, kF);
		
		pidController.setOutputRange(0, 1);

		Devices.rightEncoder.reset();
		
		pidController.setSetpoint(100 * 4096 / 600); 	// 100 rpm to ticks/100ms.

		while (isAutoActive()) 
		{
			LCD.printLine(4, "wheel encoder=%d  rpm=%d", Devices.rightEncoder.get(), Devices.rightEncoder.pidGet());

			// Use PID to determine the power applied. Should reduce power as we get close
			// to the target encoder RPM.
			
			elapsedTime = Util.getElaspedTime();
				
			pidController.calculate(Devices.rightEncoder.pidGet(), elapsedTime);
				
			power = pidController.get();
				
			Util.consoleLog("error=%.2f  power=%.2f  rpm=%d  time=%f", pidController.getError(), power, 
					Devices.rightEncoder.getRPM(), elapsedTime);
			
			Devices.RRCanTalon.set(power);
			
			Timer.delay(.100);
		}
	}
	
	private void VelocityTest2()
	{
		StringBuilder 	_sb = new StringBuilder();
		
		Util.consoleLog();		
		
		/* first choose the sensor */
		Devices.RRCanTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0,
		30);
		
		Devices.RRCanTalon.setSensorPhase(true);

		 /* set the peak, nominal outputs, and deadband */
		 Devices.RRCanTalon.configNominalOutputForward(0, 30);
		 Devices.RRCanTalon.configNominalOutputReverse(0, 30);
		 Devices.RRCanTalon.configPeakOutputForward(1, 30);
		 Devices.RRCanTalon.configPeakOutputReverse(-1, 30);

		 /* set closed loop gains in slot0 */
		 Devices.RRCanTalon.config_kF(0, 0.34, 30);
		 Devices.RRCanTalon.config_kP(0, 0.2, 30);
		 Devices.RRCanTalon.config_kI(0, 0, 30);
		 Devices.RRCanTalon.config_kD(0, 0, 30);

		 while (isAutoActive())
		 {
			 /* get gamepad axis */
			double leftYstick = Devices.leftStick.getY();
			double motorOutput = Devices.RRCanTalon.getMotorOutputPercent();
	
			/* prepare line to print */
			_sb.append("\tout:");
			_sb.append(motorOutput);
			_sb.append("\tspd:");
			_sb.append(Devices.RRCanTalon.getSelectedSensorVelocity(0));
	
			if (Devices.leftStick.getRawButton(1)) 	// Trigger.
			{
				/* Speed mode 500 rpm max */
				/*
				* 4096 Units/Rev * 500 RPM / 600 100ms/min in either direction:
				* velocity setpoint is in units/100ms
				*/
				double targetVelocity_UnitsPer100ms = leftYstick * 4096 * 500.0 / 600;
				
				Devices.RRCanTalon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
				
				/* append more signals to print when in speed mode. */
				_sb.append("\terr:");
				_sb.append(Devices.RRCanTalon.getClosedLoopError(0));
				_sb.append("\ttrg:");
				_sb.append(targetVelocity_UnitsPer100ms);
			} 
			else 
			{
			/* Percent output mode */
				Devices.RRCanTalon.set(ControlMode.PercentOutput, leftYstick);
			}
	
			Util.consoleLog(_sb.toString());
			
			_sb.setLength(0);
		
			Timer.delay(.100);
		}
	}
	
	/**
	 * Auto drive straight in set direction and power for specified encoder count. Stops
	 * with or without brakes on CAN bus drive system. Uses NavX yaw to drive straight.
	 * @param power Power applied, + is forward.
	 * @param encoderCounts Target encoder counts to move, always +.
	 * @param stop Stop stops motors at end of curve, dontStop leaves power on to flow into next move.
	 * @param brakes Brakes on or off.
	 * @param pid On is use PID to control movement, off is simple drive.
	 * @param heading Heading is measure steering yaw from last set navx target heading, angle is measure yaw
	 * from direction robot is pointing when driving starts.
	 * 
	 * Note: This routine is designed for tank drive and the P,I,D,steering gain values will likely need adjusting for each
	 * new drive base as gear ratios and wheel configuration may require different values to stop smoothly
	 * and accurately.
	 */
	private void autoDrive(double power, int encoderCounts, StopMotors stop, Brakes brakes, Pid pid, 
						    Heading heading)
	{
		double			yaw, kSteeringGain = .10, elapsedTime = 0;
		double			kP = .002, kI = 0.001, kD = 0.001;
		
		SynchronousPID	pidController = null;

		Util.consoleLog("pwr=%.2f  count=%d  stop=%s  brakes=%s  pid=%s  hdg=%s", power, encoderCounts, stop, brakes, 
						pid, heading);

		Util.checkRange(power, 1.0);
		
		if (encoderCounts <= 0) throw new IllegalArgumentException("Encoder counts < 1");
		
		if (brakes == Brakes.on)
			Devices.SetCANTalonBrakeMode(true);
		else
			Devices.SetCANTalonBrakeMode(false);
			
		Devices.wheelEncoder.reset();
		//Devices.wheelEncoder2.reset();
		
		if (robot.isClone) Timer.delay(0.3);
		
		// If not measuring yaw from current heading, reset yaw based on current direction robot is facing.
		
		if (heading == Heading.angle)
		{
			Util.consoleLog("before reset=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
			
			Devices.navx.resetYawWait(1, 500);
			
			Util.consoleLog("after reset2=%.2f  hdg=%.2f", Devices.navx.getYaw(), Devices.navx.getHeading());
		}
		
		// If using PID to control distance, configure the PID object.
		
		if (pid == Pid.on)
		{
			pidController = new SynchronousPID(kP, kI, kD);
			
			if (power < 0)
			{
				pidController.setSetpoint(-encoderCounts);
				pidController.setOutputRange(power, 0);
			}
			else
			{
				pidController.setSetpoint(encoderCounts);
				pidController.setOutputRange(0, power);
			}

			Util.getElaspedTime();
		}
		
		// Drive until we get there.
		
		while (isAutoActive() && Math.abs(Devices.wheelEncoder.get()) < encoderCounts) 
		{
			LCD.printLine(4, "wheel encoder=%d  winch encoder=%d", Devices.wheelEncoder.get(), Devices.winchEncoder.get());

			// Use PID to determine the power applied. Should reduce power as we get close
			// to the target encoder value.
			
			if (pid == Pid.on)
			{
				elapsedTime = Util.getElaspedTime();
				
				pidController.calculate(Devices.wheelEncoder.get(), elapsedTime);
				
				power = pidController.get();
				
				Util.consoleLog("error=%.2f  power2=%.2f  time=%f", pidController.getError(), power, elapsedTime);
			}

			// Yaw angle is negative if robot veering left, positive if veering right when going forward.
			
			if (heading == Heading.heading)
				yaw = Devices.navx.getHeadingYaw();
			else
				yaw = Devices.navx.getYaw();
			
			LCD.printLine(5, "yaw=%.2f", yaw);
			
			Util.consoleLog("yaw=%.2f  hdg=%.2f  rot=%.2f", yaw, Devices.navx.getHeading(), -yaw * kSteeringGain);
			
			// Note we invert sign on the angle because we want the robot to turn in the opposite
			// direction than it is currently going to correct it. So a + angle says robot is veering
			// right so we set the turn value to - because - is a turn left which corrects our right
			// drift. kSteeringGain controls how aggressively we turn to stay on course.
			
			Devices.robotDrive.curvatureDrive(power, Util.clampValue(-yaw * kSteeringGain, 1.0), false);
			
			Timer.delay(.010);
		}

		if (stop == StopMotors.stop) Devices.robotDrive.stopMotor();				
		
		Util.consoleLog("end: actual count=%d  error=%.3f  ena=%b  isa=%b", Math.abs(Devices.wheelEncoder.get()), 
				(double) Math.abs(Devices.wheelEncoder.get()) / encoderCounts, robot.isEnabled(), robot.isAutonomous());
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
	 * Note: This routine is designed for tank drive and the P,I,D,tolerance values will likely need adjusting for each
	 * new drive base as gear ratios and wheel configuration may require different values to turn smoothly
	 * and accurately.
	 */
	private void autoRotate(double power, double target, boolean usePid, boolean useHeading)
	{
		double	kP = .02, kI = 0.003, kD = 0.001, kTolerance = 1.0;
		double	elapsedTime, yaw = 0;
		
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
			
			Util.getElaspedTime();
			
			while (isAutoActive() && !pid.onTarget(kTolerance)) 
			{
				if (useHeading)
					yaw = Devices.navx.getHeadingYaw();
				else
					yaw = Devices.navx.getYaw();
				
				elapsedTime = Util.getElaspedTime();
				
				// Our target is zero yaw so we determine the difference between
				// current yaw and target and perform the PID calculation which
				// results in the speed of turn, reducing power as the difference
				// approaches zero. So our turn should slow and not overshoot. If
				// it does, the PID controller will reverse power and turn it back.
				
				pid.calculate(yaw, elapsedTime);
				
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
	 * @param stop Stop stops motors at end of curve, dontStop leaves power on to flow into next move.
	 * @param brakes On turns on brakes for end of curve, off turns them off.
	 * @param pid On uses PID controller to manage the curve slowing rotation as target is reached. Off
	 * uses the fixed curve value for whole rotation.
	 * @param heading Heading: target is a heading, angle: target is an angle from current direction.
	 * 
	 * Note: This routine is designed for tank drive and the P,I,D,tolerance values will likely need adjusting for each
	 * new drive base as gear ratios and wheel configuration may require different values to stop smoothly
	 * and accurately.
	 */
	private void autoCurve(double power, double curve, double target, StopMotors stop, Brakes brakes, Pid pid, 
						   Heading heading)
	{
		double	kP = .04, kI = 0.003, kD = 0.001, kTolerance= 1.0;
		double	elapsedTime, yaw = 0, originalCurve, power2;
		
		SynchronousPID	pidController = null;

		Util.consoleLog("pwr=%.2f  curve=%.2f  target=%.2f  stop=%s  brakes=%s  pid=%s  hdg=%s  kP=%.3f  kI=%.3f", 
						power, curve, target, stop, brakes, pid, heading, kP, kI);
		
		Util.checkRange(power, 1.0, "power");
		
		Util.checkRange(curve, 0, 1.0, "curve");
		
		if (brakes == Brakes.on)
			Devices.SetCANTalonBrakeMode(true);
		else
			Devices.SetCANTalonBrakeMode(false);

		// Reset yaw to current robot direction or target heading.
		
		if (heading == Heading.heading) 
		{
			Util.checkRange(target, 0, 359.999, "target");
			
			Devices.navx.setTargetHeading(target);
		}
		else
		{
			Util.checkRange(target, 180, "target");
			
			Devices.navx.resetYawWait(1, 500);
		}
		
		if (pid == Pid.on)
		{
			// Use PID to control power as we turn slowing as we approach target heading.
			
			pidController = new SynchronousPID(kP, kI, kD);
			
			pidController.setOutputRange(-Math.abs(curve) , Math.abs(curve));
			
			originalCurve = curve;
			
			if (heading == Heading.heading)
				pidController.setSetpoint(0);		// We are trying to get the yaw to zero.
			else
				pidController.setSetpoint(target);	// We are trying to get to the target yaw.
			
			// The PID class needs delta time between calls to calculate the I term.
			
			Util.getElaspedTime();
			
			while (isAutoActive() && !pidController.onTarget(kTolerance)) 
			{
				elapsedTime = Util.getElaspedTime();
				
				if (heading == Heading.heading)
					yaw = Devices.navx.getHeadingYaw();
				else
					yaw = Devices.navx.getYaw();

				// Our target is zero yaw so we determine the difference between
				// current yaw and target and perform the PID calculation which
				// results in the speed (curve) of turn, reducing curve as the difference
				// approaches zero. So our turn should slow and not overshoot. If
				// it does, the PID controller will reverse curve and turn it back.
				
				pidController.calculate(yaw, elapsedTime);
				
				curve = pidController.get();
				
				// When quick turn is false, power is constant, curve is fed to the
				// rate of turn parameter. PID controller takes care of the sign, that 
				// is the left/right direction of the turn. If stopping at end of curve
				// we use the PID calculated curve (which is getting smaller) to also
				// reduce motor power for smooth stop.
				
				if (stop == StopMotors.stop)
					power2 = power * Math.abs(curve / originalCurve);
				else
					power2 = power;
				
				Devices.robotDrive.curvatureDrive(power2, curve, false);
				
				Util.consoleLog("power=%.2f  hdg=%.2f  yaw=%.2f  curve=%.2f  err=%.2f  time=%f  ena=%b", power2, Devices.navx.getHeading(), 
						yaw, curve, pidController.getError(), elapsedTime, robot.isEnabled());
				
				Timer.delay(.010);
			} 
		}
		else
		{
			// Simple turn, full curve until target reached.
			
			if (heading == Heading.heading)
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
		
		// Stop motors. If you don't use stop, motors will keep running.
		if (stop == StopMotors.stop) Devices.robotDrive.stopMotor();

		Util.consoleLog("after stop  hdg=%.2f  yaw=%.2f", Devices.navx.getHeading(), yaw);

		// Wait for robot to stop moving.
		Util.consoleLog("moving=%b", Devices.navx.isRotating());
		//while (isAutoActive() && Devices.navx.isRotating()) {Timer.delay(.10);}
		//Util.consoleLog("moving=%b", Devices.navx.isRotating());
		
		Util.consoleLog("end hdg=%.2f  yaw=%.2f  ena=%b", Devices.navx.getHeading(), yaw, robot.isEnabled());
	}
	
	/**
	 * Drive in S curve, curve one direction, drive straight, curve back to starting heading.
	 * @param power Speed to drive + is forward.
	 * @param curve Rate of rotation (0..1.0), always +. PID controller will turn left/right
	 * as needed to reach target heading.
	 * @param target Heading to turn to 0..359.
	 * @param straightEncoderCounts Length of straight led between curves.
	 */
	private void autoSCurve(double power, double curve, double target, int straightEncoderCounts)
	{
		double	saveHeading = Devices.navx.getHeading();
		
		Util.consoleLog("pwr=%.2f  curve=%.2f  target=%.2f  counts=%d", power, curve, target, straightEncoderCounts);
		
		Devices.SetCANTalonRampRate(0.5);	

		// We start out driving in a curve until we have turned to the desired heading.
		// Then we drive straight the desired distance then curve back to starting
		// heading. PID controller in autoCurve will determine the sign of curve.
		
		autoCurve(power, curve, target, StopMotors.dontStop, Brakes.off, Pid.on, Heading.heading);
		
		autoDrive(power, straightEncoderCounts, StopMotors.dontStop, Brakes.off, Pid.off, Heading.heading);
		
		autoCurve(power, curve, saveHeading, StopMotors.stop, Brakes.on, Pid.on, Heading.heading);
	}
	
	private enum Brakes
	{
		off,
		on
	}
	
	private enum Pid
	{
		off,
		on
	}
	
	private enum Heading
	{
		angle,
		heading
	}
	
	private enum StopMotors
	{
		dontStop,
		stop
	}
}