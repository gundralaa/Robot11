
package Team4450.Robot11;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;
import Team4450.Robot11.Lift.LiftHeight;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Autonomous extends GamePhase
{
	private final int	program = (int) SmartDashboard.getNumber("AutoProgramSelect",0);

	Autonomous(Robot robot)
	{
		super(robot);
		Util.consoleLog();	
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
		Util.consoleLog("Alliance=%s, Location=%d, Program=%d, FMS=%b, msg=%s, match", robot.alliance.name(), robot.location, program, 
				Devices.ds.isFMSAttached(), robot.gameMessage);
		LCD.printLine(2, "Alliance=%s, Location=%d, FMS=%b, Program=%d, msg=%s", robot.alliance.name(), robot.location, 
				Devices.ds.isFMSAttached(), program, robot.gameMessage);

		Devices.robotDrive.setSafetyEnabled(false);

		Devices.lowGear();
		Devices.resetServo();
		
		Lift.getInstance(robot).extendWrist();
		Lift.getInstance(robot).closeClaw();
		Lift.getInstance(robot).setWinchBrake(false);

		// Initialize encoder.
		Devices.driveEncoder1.reset();
		Devices.driveEncoder2.reset();
		Devices.winchEncoder.reset();

		// Set gyro/NavX to heading 0.
		//robot.gyro.reset();
		Devices.navx.resetYaw();

		// Wait to start motors so gyro will be zero before first movement.
		Timer.delay(.50);
		
		Devices.SetCANTalonBrakeMode(true);

		switch (program)
		{
		case 0:		// No auto program.
			break;
		case 1:     // Move forward side, w/o scoring.
			moveForwardSide();
			break;
		case 2:     // Move forward center, w/o scoring.
			moveForwardCenter();
			break;
		case 3:     // Center w/scoring.
			switch (robot.gameMessage.charAt(0)) {
			case 'L':
				scoreCenterLeft();
				break;
			case 'R':
				scoreCenterRight();
				break;
			default:
				moveForwardCenter();
				break;
			}
			break;
		case 4:     //Left w/scoring.
			switch(robot.gameMessage.charAt(0)) {
			case 'L':
				scoreLeftSwitch();
				break;
			default:
				moveForwardSide();
			}
			break;
		case 5:     //Right w/scoring.
			switch(robot.gameMessage.charAt(0)) {
			case 'R':
				scoreRightSwitch();
				break;
			default:
				moveForwardSide();
			}
			break;

		case 6:		//Center w/scoring alt method
			switch(robot.gameMessage.charAt(0)) {
			case 'R':
				scoreCenterRightAlt();
				break;
			case 'L':
				scoreCenterLeftAlt();
				break;
			default: 
				moveForwardCenter();
				break;
			}
			break;
			
		case 7:		//Center w/scoring curve method
			switch(robot.gameMessage.charAt(0)) {
			case 'R':
				scoreCenterRightCurve();
				break;
			case 'L':
				scoreCenterLeftCurve();
				break;
			default: 
				moveForwardCenter();
				break;
			}
			break;
			
		case 8:
			switch(robot.gameMessage.charAt(0)) {
			case 'R':
				holly2Cube();
				break;
			case 'L':
				moveForwardCenter();
				break;
			default:
				moveForwardCenter();
				break;	
			}
			break;
		}

		Util.consoleLog("end");
	}

	private void scoreCenterLeft() {
		moveLift(LiftHeight.SWITCH);
		autoDrive(-.4,924,true); //Move out a little
		autoRotate(.5, 90); //Turn to the left
		autoDrive(-.6, 928, true); //Align with switch
		autoRotate(-.5, 90); //Face switch
		autoDrive(-.4, 880, true); //Go to switch
		ejectCube(); //... Take a guess
		//autoDrive(.3, 200, true); //Go back a tiny bit
		//moveLift(LiftHeight.GROUND); //Lower lift
	}

	private void scoreCenterRight() {
		moveLift(LiftHeight.SWITCH);
		autoDrive(-.4,924,true); //Move out a little
		autoRotate(-.5, 90); //Turn to the right
		autoDrive(-.6, 900, true); //Align with switch
		autoRotate(.5, 90); //Face switch
		autoDrive(-.4, 880, true); //Go to switch
		ejectCube(); //... Take a guess
		//autoDrive(.3, 200, true); //Go back a tiny bit
		//moveLift(LiftHeight.GROUND); //Lower lift
	}

	private void scoreLeftSwitch() {
		moveLift(LiftHeight.SWITCH);
		autoDrive(-.5, 3000, true); //Move out to line up with switch
		autoRotate(-.5, 90); //Face switch
		autoDrive(-.25, 320, true); //Go towards switch
		ejectCube();
	}

	private void scoreRightSwitch() {
		moveLift(LiftHeight.SWITCH);
		autoDrive(-.5, 3000, true); //Move out to line up with switch
		autoRotate(.5, 90); //Face switch
		autoDrive(-.3, 320, true); //Go towards switch
		ejectCube();
	}

	private void moveForwardSide() {
		autoDrive(-.5, 2490, true);
	}

	private void moveForwardCenter() {
		autoDrive(-.3, 1970, true); //Move out a little
	}

	private void scoreCenterRightAlt() {
		moveLift(LiftHeight.SWITCH);
		autoDrive(-.40, 100, true); //Move forward a bit
		autoRotate(-.50, 19); //Turn to switch
		autoDrive(-.50, 1900, true); //Go there
		ejectCube(); //Eject Cube
		//autoDrive(.3, 100, true);
		//moveLift(LiftHeight.GROUND);
	}
	
	private void scoreCenterLeftAlt () {
		moveLift(LiftHeight.SWITCH);
		autoDrive(-.40, 100, true); //Move forward a bit
		autoRotate(.50, 26); //Turn to switch
		autoDrive(-.50, 2100, true); //Go there
		ejectCube(); //Eject Cube
		//autoDrive(.3, 100, true);
		//moveLift(LiftHeight.GROUND);
	}
	
	private void scoreCenterRightCurve() {
		moveLift(LiftHeight.SWITCH);
		autoSCurve(-.50, -6, 30, 950);
		ejectCube();
		autoDrive(.50, 1400, true);
		Lift.getInstance(robot).setLiftHeight(LiftHeight.GROUND);
		autoRotate(.60, 90); //Turn to center
		autoDrive(-.60, 850, true); //Go to center
		autoRotate(-.60, 90); //Turn to pile
		Lift.getInstance(robot).toggleIntakeCube(); //Get cube
		autoDrive(-.60, 350, true); //Go to front of pile
		Lift.getInstance(robot).closeClaw();
		autoDrive(.60, 300, true);
	}
	
	private void scoreCenterLeftCurve() {
		moveLift(LiftHeight.SWITCH);
		autoSCurve(-.50, 6, 35,	1050);
		ejectCube();
		autoDrive(.50, 1400, true);
		Lift.getInstance(robot).setLiftHeight(LiftHeight.GROUND);
		autoRotate(-.50, 90); //Turn to center
		autoDrive(-.50, 850, true); //Go to center
		autoRotate(.50, 90); //Turn to pile
		Lift.getInstance(robot).toggleIntakeCube(); //Get cube
		autoDrive(-.60, 350, true); //Go to front of pile
		Lift.getInstance(robot).closeClaw();
		autoDrive(.60, 300, true);
	}
	
	private void holly2Cube() { //FIXME Debug
		moveLift(LiftHeight.SWITCH);
		autoDrive(-.40, 100, true); //Move forward a bit
		autoRotate(-.50, 19); //Turn to switch
		autoDrive(-.50, 1900, true); //Go there
		ejectCube(); //Eject Cube
		Timer.delay(1);
        autoDrive(0.50, 1729, true); //Backup to pile
        Timer.delay(0.3);
        autoRotate(0.50, 41); //Turn to pile
        moveLift(LiftHeight.GROUND);
        autoDrive(-0.50, 678, true); //Go to pile
        Lift.getInstance(robot).toggleIntakeCube();
        autoDrive(0.50, 678, true); //Backup from pile
        moveLift(LiftHeight.SWITCH);
        Timer.delay(0.5);
        autoRotate(0.50, 8); //Turn to switch mostly
        autoDrive(-0.50, 1511, true); //Go most of the way
        autoRotate(0.50, 26); //Turn to switch
        autoDrive(-0.50, 913, true); //Go to switch
        ejectCube();
	}

	private void ejectCube() {	
		Util.consoleLog("Eject Cube");
		Lift.getInstance(robot).ejectCube();
	}

	private void moveLift(LiftHeight height) {
		Util.consoleLog("Raise lift " + height.name());
		Lift.getInstance(robot).setLiftHeight(height);
	}

	// Auto drive in set direction and power for specified encoder count. Stops
	// with or without brakes on CAN bus drive system. Uses gyro/NavX to go straight.

	private void autoDrive(double power, int encoderCounts, boolean enableBrakes)
	{
		int		angle;
		double	gain = .05;

		Util.consoleLog("pwr=%.2f, count=%d, brakes=%b", power, encoderCounts, enableBrakes);

		if (Robot.isComp) Devices.SetCANTalonBrakeMode(enableBrakes);

		if (Robot.isClone) Timer.delay(0.3);
		
		Devices.driveEncoder1.reset();
		Devices.driveEncoder2.reset();
		Devices.navx.resetYaw();
		
		if (power > 0) gain = -gain;

		while (isAutoActive() && Math.abs(Devices.driveEncoder1.get()) < encoderCounts) 
		{
			LCD.printLine(4, "encoder1=%d encoder2=%d", Devices.driveEncoder1.get(), Devices.driveEncoder2.get());

			// Angle is negative if robot veering left, positive if veering right when going forward.
			// It is opposite when going backward. Note that for this robot, - power means forward and
			// + power means backward.

			//angle = (int) robot.gyro.getAngle();
			angle = (int) adjustAngle(Devices.navx.getYaw());

			LCD.printLine(5, "angle=%d", angle);

			// Invert angle for backwards.

			if (power > 0) angle = -angle;

			//Util.consoleLog("adjusted yaw=%.2f heading=%.2f", adjustAngle(Devices.navx.getYaw()), Devices.navx.getHeading());

			// Note we invert sign on the angle because we want the robot to turn in the opposite
			// direction than it is currently going to correct it. So a + angle says robot is veering
			// right so we set the turn value to - because - is a turn left which corrects our right
			// drift.

			Devices.robotDrive.curvatureDrive(power, angle * gain, false);

			Timer.delay(.020);
		}

		Devices.robotDrive.tankDrive(0, 0, true);				

		Util.consoleLog("end: actual count1=%d actual count2=%d", Math.abs(Devices.driveEncoder1.get()), Math.abs(Devices.driveEncoder2.get()));
	}

	/** Auto rotate left or right the specified angle. Left/right from robots forward view.
	 * Turn right, power is -
	 * Turn left, power is +
	 * angle of rotation is always +.
	 **/
	private void autoRotate(double power, int angle)
	{
		Util.consoleLog("pwr=%.2f  angle=%d", power, angle);

		Devices.navx.resetYaw();

		Util.consoleLog("AutoRotate: " + power);
		Devices.robotDrive.tankDrive(power, -power);

		while (isAutoActive() && Math.abs((int) adjustAngle(Devices.navx.getYaw())) < angle) {Timer.delay(.020);} 

		Util.consoleLog("AutoRotate: Stop. Adjusted Angle traveled: " + adjustAngle(Devices.navx.getYaw()));
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
		
		while (isAutoActive() && Math.abs((int) adjustAngle(Devices.navx.getYaw())) < targetAngle) 
		{
			LCD.printLine(6, "angle=%.2f adjusted angle=%.2f", Devices.navx.getYaw(), adjustAngle(Devices.navx.getYaw()));
			Timer.delay(.020);
		}
		
		autoDrive(power, straightEncoderCounts, false);

		Devices.navx.resetYaw();
		
		Devices.robotDrive.curvatureDrive(power*.7, -curve * gain, false);
		
		while (isAutoActive() && Math.abs((int) adjustAngle(Devices.navx.getYaw())) < (targetAngle-10) && ((Devices.ds.getMatchType() != MatchType.None) ? Devices.ds.getMatchTime() > 5 : true)) 
		{
			LCD.printLine(6, "angle=%.2f adjusted angle=%.2f", Devices.navx.getYaw(), adjustAngle(Devices.navx.getYaw()));
			Timer.delay(.020);
		}
		
		Devices.SetCANTalonBrakeMode(true);

		Devices.robotDrive.tankDrive(0, 0, true);
	}
	
	public float adjustAngle(float angle) {
		if (Robot.isClone) return angle*(18.0f/15.0f);
		else return angle;
	}
}