
package Team4450.Robot11;

import java.lang.Math;

import Team4450.Lib.*;
import Team4450.Lib.GamePad.GamePadButton;
import Team4450.Lib.GamePad.GamePadEvent;
import Team4450.Lib.GamePad.GamePadEventListener;
import Team4450.Lib.JoyStick.*;
import Team4450.Lib.LaunchPad.*;
import Team4450.Robot11.Lift.BarReleaseState;
import Team4450.Robot11.Lift.LiftHeight;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Teleop extends GamePhase
{
	public  GamePad			xboxController;
	private boolean				autoTarget, invertDrive, altDriveMode;
	private Vision				vision;

	// Constructor.

	Teleop(Robot robot)
	{
		super(robot);
		Util.consoleLog();

		vision = Vision.getInstance(robot);
	}

	// Free all objects that need it.

	void dispose()
	{
		Util.consoleLog();

		if (xboxController != null) xboxController.dispose();
	}
	
	public float adjustAngle(float angle) {
		if (Robot.isClone) return angle*(18.0f/15.0f);
		else return angle;
	}

	void execute()
	{
		double	rightY = 0, leftY = 0, utilY = 0, rightX = 0, leftX = 0;
		double	gain = .01;
		boolean	steeringAssistMode = false;
		int		angle;

		// Motor safety turned off during initialization.
		Devices.robotDrive.setSafetyEnabled(false);
		Devices.resetServo();
		Devices.lowGear();
		Lift.getInstance(robot).setWinchBrake(false);
		
		Devices.SetCANTalonBrakeMode(false); //For 2018 force coast

		Util.consoleLog();

		LCD.printLine(1, "Mode: OperatorControl");
		LCD.printLine(2, "Alliance=%s, Start=%d, FMS=%b", robot.alliance.name(), robot.location, Devices.ds.isFMSAttached());

		// Configure LaunchPad and Joystick event handlers.
		Util.consoleLog("Begin Joystick Setup");
		
		xboxController = new GamePad(Devices.xboxController, "XboxController", this);
		xboxController.addGamePadEventListener(new GamePadListener());
		xboxController.Start();
		
		Util.consoleLog("Joystick Setup Finished.");
		
		if (Robot.isComp) Devices.SetCANTalonBrakeMode(true);

		// Set gyro/Navx to heading 0.
		//robot.gyro.reset();
		Devices.navx.resetYaw();

		Devices.navx.setHeading(90);

		// Reset encoder.
		Devices.driveEncoder1.reset();
		Devices.driveEncoder2.reset();

		// Motor safety turned on.
		Devices.robotDrive.setSafetyEnabled(true);
		
		// Driving loop runs until teleop is over.

		while (robot.isEnabled() && robot.isOperatorControl())
		{
			// Get joystick deflection and feed to robot drive object
			// using calls to our JoyStick class.

			rightY = stickLogCorrection(xboxController.GetRightY());	// fwd/back
			leftY = stickLogCorrection(xboxController.GetLeftY());	// fwd/back

			rightX = stickLogCorrection(xboxController.GetRightX());	// left/right
			leftX = stickLogCorrection(xboxController.GetLeftX());	// left/right

			utilY = xboxController.GetRightTrigger() - xboxController.GetLeftTrigger();

			LCD.printLine(3, "limit switch= %b", Devices.winchLimitSwitch.get());
			LCD.printLine(4, "leftY=%.4f  rightY=%.4f  utilY=%.4f", leftY, rightY, utilY);
			LCD.printLine(6, "yaw=%.2f, adj yaw=%.2f, total=%.2f, rate=%.2f, hdng=%.2f", Devices.navx.getYaw(), adjustAngle(Devices.navx.getYaw()), Devices.navx.getTotalYaw(), 
					Devices.navx.getYawRate(), Devices.navx.getHeading());
			LCD.printLine(7, "winchEncoder=%d", Devices.winchEncoder.get());
			LCD.printLine(8, "pressureV=%.2f  psi=%d", robot.monitorCompressorThread.getVoltage(), robot.monitorCompressorThread.getPressure());
			LCD.printLine(9, "Drive Encoders: 1:%d 2:%d", Devices.driveEncoder1.get(), Devices.driveEncoder2.get());
			
			if ((Robot.isClone ? !Devices.winchLimitSwitch.get() : Devices.winchLimitSwitch.get()) && !Lift.getInstance(robot).isAutoLifting()) {
				Devices.winchEncoder.reset();
			} else if (Lift.getInstance(robot).getPID().get() < 0 && (Robot.isClone ? !Devices.winchLimitSwitch.get() : Devices.winchLimitSwitch.get())) {
				Lift.getInstance(robot).stopAutoLift();
				Devices.winchEncoder.reset();
			}
			
			// Set wheel motors.
			// Do not feed JS input to robotDrive if we are controlling the motors in automatic functions.

			//if (!autoTarget) robot.robotDrive.tankDrive(leftY, rightY);

			// Two drive modes, full tank and alternate. Switch on right stick trigger.

			if (!autoTarget) 
			{
				if (altDriveMode)
				{	// normal tank with straight drive assist when sticks within 10% of each other.
					if (leftRightEqual(leftY, rightY, 10) && Math.abs(rightY) > .50)
					{
						if (!steeringAssistMode) Devices.navx.resetYaw();

						// Angle is negative if robot veering left, positive if veering right when going forward.
						// It is opposite when going backward. Note that for this robot, - power means forward and
						// + power means backward.

						angle = (int) adjustAngle(Devices.navx.getYaw());

						LCD.printLine(5, "angle=%d", angle);

						// Invert angle for backwards.

						if (rightY > 0) angle = -angle;

						//Util.consoleLog("angle=%d", angle);

						// Note we invert sign on the angle because we want the robot to turn in the opposite
						// direction than it is currently going to correct it. So a + angle says robot is veering
						// right so we set the turn value to - because - is a turn left which corrects our right
						// drift.

						Devices.robotDrive.curvatureDrive(rightY, angle * gain, true);

						steeringAssistMode = true;
					}
					else
					{
						steeringAssistMode = false;
						Devices.robotDrive.tankDrive(leftY, rightY);		// Normal tank drive.
					}

					SmartDashboard.putBoolean("Overload", steeringAssistMode);
				}
				else
					Devices.robotDrive.tankDrive(leftY, rightY);		// Normal tank drive.
			}

			Lift.getInstance(robot).setMotor(utilY);
			
			// Update the robot heading indicator on the DS.

			SmartDashboard.putNumber("Gyro", Devices.navx.getHeading());
			
			Devices.updateNetworkTables();

			// End of driving loop.

			Timer.delay(.020);	// wait 20ms for update from driver station.
		}

		// End of teleop mode.

		Util.consoleLog("end");
	}

	private boolean leftRightEqual(double left, double right, double percent)
	{
		//if (left == right) return true;

		if (Math.abs(left - right) <= (1 * (percent / 100))) return true;

		return false;
	}

	// Custom base logarithm.
	// Returns logarithm base of the value.

	private double baseLog(double base, double value)
	{
		return Math.log(value) / Math.log(base);
	}

	// Map joystick y value of 0.0 to 1.0 to the motor working power range of approx 0.5 to 1.0 using
	// logarithmic curve.

	private double stickLogCorrection(double joystickValue)
	{
		double base = Math.pow(2, 1/3) + Math.pow(2, 1/3);

		if (joystickValue > 0)
			joystickValue = baseLog(base, joystickValue + 1);
		else if (joystickValue < 0)
			joystickValue = -baseLog(base, -joystickValue + 1);

		return joystickValue;
	}



	// Handle GamePad control events.

	public class GamePadListener implements GamePadEventListener 
	{
		public void ButtonDown(GamePadEvent gamePadEvent) 
		{
			GamePadButton	control = gamePadEvent.button;

			Util.consoleLog("%s, latchedState=%b", control.id.name(),  control.latchedState);

			switch(control.id)
			{
			//Example of case:
			/*
			case BUTTON_NAME_HERE:
				if (control.latchedState)
					DoOneThing();
				else
					DoOtherThing();
				break;
			 */
			case LEFT_BUMPER: //Trigger forklift drop
				Lift.getInstance(robot).setBarRelease(BarReleaseState.EXTENDPIN);
				break;
				
			case RIGHT_BUMPER: //Toggle Camera
				CameraFeed.getInstance().ChangeCamera();
				break;
				
			case LEFT_STICK: //Change Gear
				Devices.toggleGear();
				break;
				
			case RIGHT_STICK: //Toggle Winch Break
				if (Devices.ds.getMatchTime() <= 45) 
					Lift.getInstance(robot).setWinchBrake(control.latchedState);
				break;
				
			case BACK: //Toggle Kid Mode
				//Devices.toggleKidMode(); //TODO Add Kid Mode
				break;
				
			case START: //Toggle limit switch override.
				Lift.getInstance(robot).toggleOverride();
				break;
				
			case A: //Toggle claw
				Lift.getInstance(robot).toggleClaw();
				break;
				
			case B: //Toggle Auto Intake
				Lift.getInstance(robot).toggleIntakeCube();
				break;
				
			case X: //Toggle Wrist
				Lift.getInstance(robot).toggleWrist();
				break;
				
			case Y: //Spit cube manually
				if (control.latchedState)
					Lift.getInstance(robot).setMotor(-.5);
				else
					Lift.getInstance(robot).setMotor(0);
				
			default:
				Util.consoleLog("Unassigned button pressed: "+ control.id.name());
				break;
			}
		}

		public void ButtonUp(GamePadEvent gamePadEvent) 
		{
			//Util.consoleLog("%s, latchedState=%b", launchPadEvent.control.name(),  launchPadEvent.control.latchedState);
		}
	}
}
