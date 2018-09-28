
package Team4450.Robot11;

import java.lang.Math;

import Team4450.Lib.*;
import Team4450.Lib.GamePad.GamePadButtonIDs;
import Team4450.Lib.GamePad.GamePadEvent;
import Team4450.Lib.GamePad.GamePadEventListener;
import Team4450.Lib.JoyStick.*;
import Team4450.Lib.LaunchPad.*;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

class Teleop
{
	private final Robot 		robot;
	public  GamePad             gamePad;
	private boolean				autoTarget, invertDrive, altDriveMode;
	private Vision				vision;

	// Constructor.

	Teleop(Robot robot)
	{
		Util.consoleLog();

		this.robot = robot;

		vision = Vision.getInstance(robot);
	}

	// Free all objects that need it.

	void dispose()
	{
		Util.consoleLog();
		if (gamePad != null) gamePad.dispose();
	}

	void OperatorControl()
	{
		double	rightY = 0, leftY = 0, utilX = 0, rightX = 0, leftX = 0;
		double	gain = .01;
		boolean	steeringAssistMode = false;
		int		angle;

		// Motor safety turned off during initialization.
		Devices.robotDrive.setSafetyEnabled(false);

		Util.consoleLog();

		LCD.printLine(1, "Mode: OperatorControl");
		LCD.printLine(2, "All=%s, Start=%d, FMS=%b", robot.alliance.name(), robot.location, Devices.ds.isFMSAttached());

		
		gamePad = new GamePad(Devices.gamePad, "GamePad", this);
		gamePad.AddButton(GamePadButtonIDs.A);
		gamePad.AddButton(GamePadButtonIDs.B);
		gamePad.addGamePadEventListener(new GamePadListener());
		gamePad.Start();

		// Tighten up dead zone for smoother climber movement.
		//utilityStick.deadZone = .05;

		// Set CAN Talon brake mode by rocker switch setting.
		// We do this here so that the Utility stick thread has time to read the initial state
		// of the rocker switch.
		//if (robot.isComp) Devices.SetCANTalonBrakeMode(lpControl.latchedState);

		// Set gyro/Navx to heading 0.
		//robot.gyro.reset();
		Devices.navx.resetYaw();

		Devices.navx.setHeading(90);

		// Reset encoder.
		//Devices.encoder.reset();

		// Motor safety turned on.
		Devices.robotDrive.setSafetyEnabled(true);
		GearTrain.setLow();

		// Driving loop runs until teleop is over.

		while (robot.isEnabled() && robot.isOperatorControl())
		{
			// Get joystick deflection and feed to robot drive object
			// using calls to our JoyStick class.

			rightY = - stickLogCorrection(gamePad.GetRightY());	// fwd/back
			leftY =  - stickLogCorrection(gamePad.GetLeftY());	// fwd/back

			rightX = stickLogCorrection(gamePad.GetRightX());	// left/right
			leftX = stickLogCorrection(gamePad.GetRightX());	// left/right

			LCD.printLine(4, "leftY=%.4f  rightY=%.4f  utilX=%.4f", leftY, rightY, utilX);
			LCD.printLine(6, "yaw=%.2f, total=%.2f, rate=%.2f, hdng=%.2f", Devices.navx.getYaw(), Devices.navx.getTotalYaw(), 
					Devices.navx.getYawRate(), Devices.navx.getHeading());
			LCD.printLine(8, "pressureV=%.2f  psi=%d", robot.monitorCompressorThread.getVoltage(), robot.monitorCompressorThread.getPressure());

			// Set wheel motors.
			// Do not feed JS input to robotDrive if we are controlling the motors in automatic functions.

			//if (!autoTarget) robot.robotDrive.tankDrive(leftY, rightY);

			// Two drive modes, full tank and alternate. Switch on right stick trigger.

			if (!autoTarget) 
			{
				Devices.robotDrive.tankDrive(leftY, rightY);		// Normal tank drive.
			}

			// Update the robot heading indicator on the DS.

			SmartDashboard.putNumber("Gyro", Devices.navx.getHeading());

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
	
	public static class GamePadListener implements GamePadEventListener{

		@Override
		public void ButtonDown(GamePadEvent arg0) {
			// TODO Auto-generated method stub
			switch(arg0.button.id) 
			{
				case A:
					if(GearTrain.isLowSpeed()) {
						GearTrain.setHigh();
					} else {
					}
				case B:
					if(GearTrain.isHighSpeed()) {
						GearTrain.setLow();
					} else {
					}
			}
			
			
		}

		@Override
		public void ButtonUp(GamePadEvent arg0) {
			// TODO Auto-generated method stub
			
		}
		
	}

}
