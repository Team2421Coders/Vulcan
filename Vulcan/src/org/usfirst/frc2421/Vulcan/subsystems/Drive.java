// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2421.Vulcan.subsystems;

import org.usfirst.frc2421.Vulcan.Robot;
import org.usfirst.frc2421.Vulcan.RobotMap;
import org.usfirst.frc2421.Vulcan.commands.*;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.GenericHID.Hand;


/**
 *
 */
public class Drive extends Subsystem {

	static double speedMult = 1;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    private final static SpeedController driveLeft1 = RobotMap.driveDriveLeft1;
    private final static SpeedController driveLeft2 = RobotMap.driveDriveLeft2;
    private final static SpeedController driveRight1 = RobotMap.driveDriveRight1;
    private final static SpeedController driveRight2 = RobotMap.driveDriveRight2;
    private final RobotDrive driveSystem = RobotMap.driveDriveSystem;
    public static XboxController xbc = RobotMap.xbc;
    private static double xValue;
    private static double yValue;
    private static double deadzone = .4;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS


    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new DriveCommand());

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    public static void setLeft(double x){
    	driveLeft1.set(x*speedMult);
    	driveLeft2.set(x*speedMult);
    }
    public static void setRight(double x){
    	driveRight1.set(-x*speedMult);
    	driveRight2.set(-x*speedMult);
    }
    //CONTROLLER STUFF
    public static void joysticks(){
    	if(xbc.getX(GenericHID.Hand.kLeft) < -deadzone || xbc.getX(GenericHID.Hand.kLeft) > deadzone){
        	xValue = xbc.getX(GenericHID.Hand.kLeft) * .75;
        }else{
        	xValue = 0;
        }
    	if(xbc.getY(GenericHID.Hand.kRight) < -deadzone || xbc.getY(GenericHID.Hand.kRight) > deadzone){
        	yValue = -xbc.getY(GenericHID.Hand.kRight);
        }else{
        	yValue = 0;
        }
    	Drive.setLeft((yValue+xValue) * .5);
    	Drive.setRight((yValue-xValue) * .5);
    }
    public static void buttonA(){
    	if(xbc.getAButton())
    	{
    		Command auto = new AutonomousCommand();
    		auto.start();
    	}
    
//    	double speed = .3;
//    	boolean isFinished = false;
//    	while(xValue == 0 && yValue == 0||!isFinished){
//    	try{
//    		double c = 0.0025;
//    		double d = .00075;
//    		double avgSize = (Robot.vision.getNumberArray("size", Robot.def)[0]+Robot.vision.getNumberArray("size", Robot.def)[1])/2;
//    		double midpoint = (Robot.vision.getNumberArray("x", Robot.def)[0]+Robot.vision.getNumberArray("x", Robot.def)[1])/2;
//    		double ds = (320-midpoint)*c*avgSize*d;
//    		System.out.println(ds);
//    		Drive.setLeft(speed+ds);
//    		Drive.setRight(speed-ds);
//    	}
//    	catch(Exception ex){
//    		isFinished = true;
//    	}
//    	if(Robot.vision.getNumberArray("size", Robot.def).length>0)
//    		isFinished = Robot.vision.getNumberArray("size", Robot.def)[0]>100.0;
//        else
//        	
//        	isFinished = true;
//    	}
    }
    public static void buttonB(){
    	
    }
    public static void buttonX(){
    	
    }
    public static void buttonY(){
    	
    }    
	
    public static void buttons(){
    	Drive.buttonA();
    	Drive.buttonB();
    	Drive.buttonX();
    	Drive.buttonY();
    }
}

