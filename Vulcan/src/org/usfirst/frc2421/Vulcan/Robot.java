// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2421.Vulcan;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.usfirst.frc2421.Vulcan.commands.*;
import org.usfirst.frc2421.Vulcan.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

    Command autonomousCommand;

    public static OI oi;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static Drive drive;
    public static Gear gear;
    public static Climb climb;
    public static double[] def = {-1};
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static NetworkTable vision;
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    RobotMap.init();
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        drive = new Drive();
        gear = new Gear();
        climb = new Climb();
        
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();
        // instantiate the command used for the autonomous period
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        autonomousCommand = new AutonomousCommand();
        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
        
		NetworkTable.setServerMode();
		vision = NetworkTable.getTable("GRIP/myBlobsReport");
		
		RobotMap.flashlight.set(Relay.Value.kForward);
		
		
		
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture(0);
		camera.setResolution(640, 480);
		UsbCamera camera2 = CameraServer.getInstance().startAutomaticCapture(1);
		camera2.setResolution(640, 480);
		//CvSink cvSink = CameraServer.getInstance().getVideo();
		
		/*
		new Thread(() -> {

			//CameraServer.getInstance().addAxisCamera("10.24.21.11");
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(640, 480);


			CvSink cvSink = CameraServer.getInstance().getVideo();
			//CvSource outputStream = CameraServer.getInstance().putVideo("Processed", 640, 480);

			Mat source = new Mat();
			//Mat output = new Mat();
			
			double[] def = {-1.0};

			while(!Thread.interrupted()) {
				cvSink.grabFrame(source);
				source.copyTo(output);
				for(int i = 0; i < vision.getNumberArray("x",def).length; i++)
				{
					Imgproc.circle(output, new Point(vision.getNumberArray("x", def)[i], vision.getNumberArray("y", def)[i]), 
							(int) vision.getNumberArray("size", def)[i], new Scalar(255,255,255), 10);
				}
				outputStream.putFrame(output);
				
				
				
				//System.out.println(table.getNumber("X", 0));
			}
		}).start();*/
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){

    }

    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    public void autonomousInit() {
        // schedule the autonomous command (example)
    	//RobotMap.flashlight.setDirection(Relay.Direction.kForward);
        RobotMap.flashlight.set(Relay.Value.kOn);
        if (autonomousCommand != null) autonomousCommand.start();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        RobotMap.servo1.set(.5);
    }

    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
    	//RobotMap.flashlight.setDirection(Relay.Direction.kForward);
        RobotMap.flashlight.set(Relay.Value.kOn);
        if (autonomousCommand != null) autonomousCommand.cancel();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
    }
}
