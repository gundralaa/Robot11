package Team4450.Robot11;

import Team4450.Lib.Util;

public class GamePhase {
	final Robot 		robot;
	
	GamePhase(Robot robot) {
		this.robot = robot;
	}
	
	public Robot getRobot() {
		return robot;
	}
	
	void dispose() {
		Util.consoleLog("Default dispose called! Overwrite this!");
	}
	
	void execute() {
		Util.consoleLog("Default execute called! Overwrite this!");
	}
}
