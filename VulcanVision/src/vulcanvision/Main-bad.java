package vulcanvision;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.opencv.core.Mat;
import org.opencv.core.MatOfKeyPoint;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;

import vulcanvision.VulcanVision;

public class Main {

	NetworkTable table;

	public static void main(String[] args) {
		// TODO Auto-generated method stub
		(new Main()).run();
	}

	public void run()
	{
		NetworkTable.setClientMode();
		NetworkTable.setIPAddress("10.24.21.230");
		table = NetworkTable.getTable("VulcanVision");
		

		CameraServer.getInstance().addAxisCamera("10.24.21.11");
		CvSink sink = CameraServer.getInstance().getVideo();
		Mat source = new Mat();
		VulcanVision vv = new VulcanVision();
		MatOfKeyPoint m = new MatOfKeyPoint();

		while(true)
		{
			sink.grabFrame(source);
			vv.process(source);
			m = vv.findBlobsOutput();
			System.out.println(m.dump());
			table.putNumber("x", m.get(1, 1)[0]);
			m.col(0).
			try
			{
				Thread.sleep(1000);
			}
			catch ( Exception ex)
			{
				System.exit(0);
			}
		}
		//TODO parse m
		//table.putNumberArray("x", m.)


	}
}


