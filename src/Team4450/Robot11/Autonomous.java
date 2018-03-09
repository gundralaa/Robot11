
package Team4450.Robot11;

import Team4450.Lib.*;
import Team4450.Robot11.Devices;
import Team4450.Robot11.Lift.LiftHeight;
import edu.wpi.first.wpilibj.Timer;
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
		Util.consoleLog("Alliance=%s, Location=%d, Program=%d, FMS=%b, msg=%s", robot.alliance.name(), robot.location, program, 
				Devices.ds.isFMSAttached(), robot.gameMessage);
		LCD.printLine(2, "Alliance=%s, Location=%d, FMS=%b, Program=%d, msg=%s", robot.alliance.name(), robot.location, 
				Devices.ds.isFMSAttached(), program, robot.gameMessage);

		Devices.robotDrive.setSafetyEnabled(false);

		lowGear();
		Devices.resetServo();

		// Initialize encoder.
		Devices.driveEncoder1.reset();
		Devices.driveEncoder2.reset();
		Devices.winchEncoder.reset();

		// Set gyro/NavX to heading 0.
		//robot.gyro.reset();
		Devices.navx.resetYaw();

		// Wait to start motors so gyro will be zero before first movement.
		Timer.delay(.50);

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
			case 'R':
				//if (robot.gameMessage.charAt(1) == 'L') {
				//	scoreLeftScale();
				//} else {
				//moveForwardSide(); //Removed Scale functionality
				//}
				moveForwardAndTurnLeft();
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
			case 'L':
				//if (robot.gameMessage.charAt(1) == 'R') {
				//	scoreRightScale();
				//} else {
				//moveForwardSide(); //Removed Scale functionality
				//}
				moveForwardAndTurnRight();
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
		}

		Util.consoleLog("end");
	}

	private void scoreCenterLeft() { //FIXME Power need configuring.
		moveLift(LiftHeight.SWITCH);
		autoDrive(-.5,924,true); //Move out a little
		autoRotate(.5, 90); //Turn to the left
		autoDrive(-.6, 928, true); //Align with switch
		autoRotate(-.5, 90); //Face switch
		autoDrive(-.4, 880, true); //Go to switch
		ejectCube(); //... Take a guess
		autoDrive(.3, 200, true); //Go back a tiny bit
		moveLift(LiftHeight.GROUND); //Lower lift
	}

	private void scoreCenterRight() { //FIXME Power need configuring.
		moveLift(LiftHeight.SWITCH);
		autoDrive(-.5,924,true); //Move out a little
		autoRotate(-.5, 90); //Turn to the right
		autoDrive(-.6, 900, true); //Align with switch
		autoRotate(.5, 90); //Face switch
		autoDrive(-.4, 880, true); //Go to switch
		ejectCube(); //... Take a guess
		autoDrive(.3, 200, true); //Go back a tiny bit
		moveLift(LiftHeight.GROUND); //Lower lift
	}

	private void scoreLeftSwitch() {
		moveLift(LiftHeight.SWITCH);
		autoDrive(-.5, 3180, true); //Move out to line up with switch
		autoRotate(-.5, 90); //Face switch
		autoDrive(-.25, 320, true); //Go towards switch
		ejectCube();
	}

	private void scoreRightSwitch() {
		moveLift(LiftHeight.SWITCH);
		autoDrive(-.5, 3180, true); //Move out to line up with switch
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

	private void moveForwardAndTurnRight() {
		autoDrive(-.5, 4600, true); //-done
		autoRotate(.5, 90);
		autoDrive(-.5, 1470, true); //-done
	}

	private void moveForwardAndTurnLeft() {
		autoDrive(-.5, 4600, true); //-done
		autoRotate(-.5, 90);
		autoDrive(-.5, 1470, true); //-done
	}
	
	private void scoreCenterRightAlt() {
		moveLift(LiftHeight.SWITCH);
		autoDrive(-.40, 100, true); //Move forward a bit
		autoRotate(.50, 16); //Turn to switch
		autoDrive(-.60, 1900, true); //Go there
		ejectCube(); //Eject Cube
		autoDrive(.3, 100, true);
		moveLift(LiftHeight.GROUND);
	}
	
	private void scoreCenterLeftAlt () {
		moveLift(LiftHeight.SWITCH);
		autoDrive(-.40, 100, true); //Move forward a bit
		autoRotate(.50, 24); //Turn to switch
		autoDrive(-.60, 2100, true); //Go there
		ejectCube(); //Eject Cube
		autoDrive(.3, 100, true);
		moveLift(LiftHeight.GROUND);
	}

	private void ejectCube() {	
		Util.consoleLog("Eject Cube");
		Lift.getInstance(robot).ejectCube();
	}

	private void moveLift(LiftHeight height) {
		Util.consoleLog("Raise lift " + height.name());
		Lift.getInstance(robot).setLiftHeight(height);
	}

	public void lowGear() {
		if (robot.isComp)
			Devices.gearShifter.SetA();
		else
			Devices.gearShifter.SetB();
	}

	public void highGear() {
		if (robot.isComp)
			Devices.gearShifter.SetB();
		else
			Devices.gearShifter.SetA();
	}

	//TODO Will need modification to work.

	// Auto drive in set direction and power for specified encoder count. Stops
	// with or without brakes on CAN bus drive system. Uses gyro/NavX to go straight.

	private void autoDrive(double power, int encoderCounts, boolean enableBrakes)
	{
		int		angle;
		double	gain = .03;

		Util.consoleLog("pwr=%.2f, count=%d, brakes=%b", power, encoderCounts, enableBrakes);

		if (robot.isComp) Devices.SetCANTalonBrakeMode(enableBrakes);

		Devices.driveEncoder1.reset();
		Devices.driveEncoder2.reset();
		Devices.navx.resetYaw();

		while (isAutoActive() && Math.abs(Devices.driveEncoder1.get()) < encoderCounts) 
		{
			LCD.printLine(4, "encoder1=%d encoder2=%d", Devices.driveEncoder1.get(), Devices.driveEncoder2.get());

			// Angle is negative if robot veering left, positive if veering right when going forward.
			// It is opposite when going backward. Note that for this robot, - power means forward and
			// + power means backward.

			//angle = (int) robot.gyro.getAngle();
			angle = (int) Devices.navx.getYaw();

			LCD.printLine(5, "angle=%d", angle);

			// Invert angle for backwards.

			if (power > 0) angle = -angle;

			//Util.consoleLog("angle=%d", angle);

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

		while (isAutoActive() && Math.abs((int) Devices.navx.getYaw()) < angle) {Timer.delay(.020);} 

		Util.consoleLog("AutoRotate: Stop");
		Devices.robotDrive.tankDrive(0, 0);
	}
}