package Team4450.Robot11;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import Team4450.Lib.Util;

public class Vision 
{
	private Robot robot;
	private Mat currentImage;
	private GripPowerUpBlockPipeline pipeline = new GripPowerUpBlockPipeline();
	private double blockOffset;
	private double distanceOffset;
	
	// This variable and method make sure this class is a singleton.
	
	public static Vision vision = null;
	
	public static Vision getInstance(Robot robot) 
	{
		if (vision == null) vision = new Vision(robot);
		
		return vision;
	}
	
	// This is the rest of the class.
	
	private Vision(Robot robot) 
	{
		this.robot = robot;
		
		Util.consoleLog("Vision created!");
	}
	
	boolean findBlockContours() {
		Rect blockBound = null;
		
		currentImage = robot.cameraThread.getCurrentImage();
		pipeline.process(currentImage);
		
		if(pipeline.findContoursOutput().size() > 1) {
			blockBound = Imgproc.boundingRect(pipeline.findContoursOutput().get(0));
		}
		else {
			Util.consoleLog("No Image Found");
		}
		
		return true;
	}

}
