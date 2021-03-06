// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2421.Vulcan.commands;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc2421.Vulcan.Robot;
import org.usfirst.frc2421.Vulcan.RobotMap;
import org.usfirst.frc2421.Vulcan.subsystems.Drive;
import org.usfirst.frc2421.Vulcan.subsystems.Gear;

import java.lang.Math;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 */
public class AutonomousCommand extends Command {
	public static boolean noBubbles = false;
	boolean end = false;
	private static boolean didZach;
	public Command zach = new ZachCommand();
	public Command josh = new JoshCommand();
	public static Timer time = new Timer();
	private static double currentTime = 0;
	public static double[] sizeArray;
	public static double[] sizeL3Array = {0,0,0};
	public static double[] sizeMiddleArray;
	public static double[] xMiddleArray;
	public static double[] sizeL2Array = {0,0};
	public static double[] xL3Array = {0,0,0};
	public static double[] xL2Array = {0,0};
	public static double[] yArray;
	public static double[] xArray;
	static double avgLength = 0;
	static int middleCounter = 0;
	static int arrayCounter = 0;
	static double angle = 30;
	static double counter = 0;
	static double lengthSum = 0;
	static double c;
	static double d;
	static double avgSize;
	static double midpoint;
	static double ds;
	static double sum;
	static double xCircum;
	static double distConstant = 45;
	static double speed = -.25;
	static double targetSize = 75.0;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public AutonomousCommand() {
    	

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=R	EQUIRES
    	
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	time.start();
    	Robot.camera.setExposureManual(20);
    	zach();
		System.out.println("josh");
		
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	try{
    		sizeArray = Robot.vision.getNumberArray("size", Robot.def);
    		xArray = Robot.vision.getNumberArray("x", Robot.def);
    		if(sizeArray.length == 2)
    			sizeL2Array = sizeArray;
    		if(xArray.length == 2)
    			xL2Array = xArray;
    		if(sizeL2Array.length > 1 && xL2Array.length > 1 && sizeL2Array[0]<targetSize){
				josh();
			}
			else
			{
				//System.out.println("bad");
				Drive.setLeft(0);
				Drive.setRight(0);
			}
    	}
    	catch(Exception ex){
    		noBubbles = true;
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
    	//System.out.println("FINISHED");
    	if(sizeArray.length>0)
    		return sizeArray[0]>targetSize;
        else
        	return noBubbles;
        	
    	
    }

    // Called once after isFinished returns true
    protected void end() {
    	new GearCommand().start();
    	/*if(!didZach){
    		currentTime = time.get();
    		while(time.get() < currentTime + .5){
    			Drive.setLeft(0);
    			Drive.setRight(0);
    		}
    		currentTime = time.get();
    		while(time.get() < currentTime + 1.5){
    			Drive.setLeft(-.5);
    			Drive.setRight(-.5);
    		}
    		currentTime = time.get();
    		while(time.get() < (currentTime + .6)){
				Drive.setLeft(.5);
				Drive.setRight(-.5);
			}
    		currentTime = time.get();
    		while(time.get() < currentTime + 1.5){
    			Drive.setLeft(.5);
    			Drive.setRight(.5);
    		}
    		currentTime = time.get();
    		while(time.get() < (currentTime + .6)){
				Drive.setLeft(-.5);
				Drive.setRight(.5);
			}
    		while(time.get() < currentTime + 1.5){
    			Drive.setLeft(.5);
    			Drive.setRight(.5);
    		}
    	}*/
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	
    }
    public static void updateVisionArrays(){
    	/*arrayCounter = 0;
    	middleCounter = 0;
    	sizeArray = Robot.vision.getNumberArray("size", Robot.def);
		xArray = Robot.vision.getNumberArray("x", Robot.def);
		yArray = Robot.vision.getNumberArray("y", Robot.def);
		//System.out.println("y array length: " + yArray.length);
		//System.out.println("size array length: " + sizeArray.length);
		for(int i = 0; i < yArray.length; i++){
			//System.out.println(i);
			if(yArray[i] > 140 && yArray[i] < 340){
				System.out.println(yArray[i] + " " + i + " " + yArray.length);
				arrayCounter += 1;
			}
		}
		sizeMiddleArray = new double[arrayCounter];
		xMiddleArray = new double[arrayCounter];
		for(int a = 0; a < yArray.length; a++){//experimental
			if(yArray[a] > 140 && yArray[a] < 340){
				sizeMiddleArray[middleCounter] = sizeArray[a];
				xMiddleArray[middleCounter] = xArray[a];
				middleCounter++;
			}
		}
		if(sizeMiddleArray.length == 3){
			sizeL3Array = sizeMiddleArray;
			xL3Array = xMiddleArray;
		}
		if(sizeMiddleArray.length == 2){
			sizeL2Array = sizeMiddleArray;
			xL2Array = xMiddleArray;
		}*/
    }
    public static double arrayAvgLength(){
    	/*currentTime = time.get();
    	while(time.get() < (currentTime + 2)){
    		updateVisionArrays();
    		lengthSum += sizeMiddleArray.length;
    		counter += 1;
    	}*/
    	return 1.0;
    }
    public static void josh(){
    	//System.out.println("josh");
		c = 0.015;
		d = 0.001;
		System.out.println(ds);
		avgSize = (sizeL2Array[0] + sizeL2Array[1])/2;
		midpoint = (xL2Array[0] + xL2Array[1])/2;
		ds = (midpoint-320)*c*avgSize*d;
		Drive.setLeft(speed+ds);
		Drive.setRight(speed-ds);
    }
    
    public static void zach(){
    	if(RobotMap.autoSwitchMid.get()){
    		currentTime = time.get();
    		while(time.get() < currentTime + 1.3){
    			Drive.setLeft(-.5);
    			Drive.setRight(-.5);
    		}
    		if(!RobotMap.autoSwitchLR.get()){
    			System.out.println("right");
    			currentTime = time.get();
        		while(time.get() < currentTime + .425){
        			Drive.setLeft(.5);
        			Drive.setRight(-.5);
        		}
    		}
    		if(RobotMap.autoSwitchLR.get()){
    			System.out.println("left");
    			currentTime = time.get();
        		while(time.get() < currentTime + .425){
        			Drive.setLeft(-.5);
        			Drive.setRight(.5);
        		}
    		}
    		currentTime = time.get();
    		while(time.get() < currentTime + 0.5){
    			Drive.setLeft(0);
    			Drive.setRight(0);
    		}
    	}
		/*
		Drive.setLeft(0);
		Drive.setRight(0);
    	System.out.println("zach");
    	updateVisionArrays();
    	avgLength = arrayAvgLength();
    	System.out.println(avgLength);
		if(avgLength > 2.3){
			didZach = true;
			sum = sizeL3Array[0] + sizeL3Array[1] + sizeL3Array[2];
			if(sum < 200){
    			xCircum = (angle/180)*(215-38.8*Math.log(sum))*(Math.PI);
    			
    			//right
    			if(xL3Array[0] < xL3Array[2]){
    				System.out.println("right");
    				currentTime = time.get();
    				while(time.get() < (currentTime + .45)){
    					Drive.setLeft(-.5);
    					Drive.setRight(.5);    					
    				}
    				Drive.setLeft(0);
    				Drive.setRight(0);
    				currentTime = time.get();
    				while(time.get() < currentTime + (xCircum/distConstant)){
    					Drive.setLeft(-.5);
    					Drive.setRight(-.5);
    				}
    				Drive.setLeft(0);
    				Drive.setRight(0);
    				currentTime = time.get();
    				while(time.get() < (currentTime + .6)){
    					Drive.setLeft(.5);
    					Drive.setRight(-.5);
    				}
    				Drive.setLeft(0);
    				Drive.setRight(0);
    			}
    			//left
    			else if(xL3Array[0] > xL3Array[2]){
    				System.out.println("left");
    				currentTime = time.get();
    				while(time.get() < (currentTime + .45)){
    					Drive.setLeft(.5);
    					Drive.setRight(-.5);    					
    				}
    				Drive.setLeft(0);
    				Drive.setRight(0);
    				currentTime = time.get();
    				while(time.get() < currentTime + (xCircum/distConstant)){
    					Drive.setLeft(-.5);
    					Drive.setRight(-.5);
    				}
    				Drive.setLeft(0);
    				Drive.setRight(0);
    				currentTime = time.get();
    				while(time.get() < (currentTime + .6)){
    					Drive.setLeft(-.5);
    					Drive.setRight(.5);
    				}
    				Drive.setLeft(0);
    				Drive.setRight(0);
    			}
    			currentTime = time.get();
    			while(time.get() < currentTime + 1){
    				Drive.setLeft(0);
    				Drive.setRight(0);
    			}
			}
		}*/
    }
}
