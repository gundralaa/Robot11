package Team4450.Robot11.Libs;

import java.util.ArrayList;

import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.wpilibj.vision.VisionPipeline;

public class Grip {
	
	/**
	 * Interface must be implemented by the
	 * Grip Vision Pipeline that is Auto Generated
	 * By the GRIP Program.
	 */
	public interface GripVisionPipeline extends VisionPipeline 
	{
		public ArrayList<MatOfPoint> findContoursOutput();
		
	}

}
