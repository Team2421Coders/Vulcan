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
    private static boolean toggleClimb = false;
    private boolean toggleGear = false;
    private static boolean lastX;
    private static boolean lastY;
    private static boolean lastB;
    private static boolean lastA;
    private static boolean lastTriggerR;
    private static boolean lastTriggerL;
    private static boolean lastBumperR;
    private static boolean lastBumperL;
    static int reversed = 1;
    static Command cc;
    static Command cc1;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS


    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        setDefaultCommand(new DriveCommand());

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND
        cc = new ClimbCommand();
        cc1 = new ClimbDownCommand();

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
    	if(xbc.getX(GenericHID.Hand.kRight) < -deadzone || xbc.getX(GenericHID.Hand.kRight) > deadzone){
        	xValue = xbc.getX(GenericHID.Hand.kRight) * .75;
        }else{
        	xValue = 0;
        }
    	if(xbc.getY(GenericHID.Hand.kLeft) < -deadzone || xbc.getY(GenericHID.Hand.kLeft) > deadzone){
        	yValue = -reversed * xbc.getY(GenericHID.Hand.kLeft);
        }else{
        	yValue = 0;
        }
    	Drive.setLeft((yValue+xValue) * .5);
    	Drive.setRight((yValue-xValue) * .5);
    }  
    //BUTTON HELD
    public static void buttonAHeld(){
    	
    }
    
    public static void buttonBHeld(){
    		Climb.climbStart();
    		System.out.println("B");
    }
    
    public static void buttonXHeld(){

    		Climb.climbDown();
    }
    
    public static void buttonYHeld(){
    		Gear.gearStart();
    }
    
    //BUTTON PRESSED
    public static void buttonAPressed(){
    	Command josh = new JoshCommand();
		josh.start();
    }
    
    public static void buttonBPressed(){
    	
    }
    
    public static void buttonXPressed(){
    	
    }
    
    public static void buttonYPressed(){
    	
    }
    
    //BUTTON RELEASED
    public static void buttonAReleased(){
    	
    }
    
    public static void buttonBReleased(){
    	Climb.climbEnd();
    }
    
    public static void buttonXReleased(){
    	Climb.climbEnd();    	
    }
    
    public static void buttonYReleased(){
    	Gear.gearEnd();
    }
    
    
    public static void triggerRPressed(){
    	
    }
    public static void triggerLPressed(){
    	
    }
    public static void bumperRPressed(){
    	
    }
    public static void bumperLPressed(){
    	
    }
    
public static void triggerRReleased(){
    	
    }
    public static void triggerLReleased(){
    	
    }
    public static void bumperRReleased(){
    	
    }
    public static void bumperLReleased(){
    	
    }

    public static void buttons(){
    	
    	if(xbc.getAButton()) Drive.buttonAHeld();
    	if(xbc.getBButton()) Drive.buttonBHeld();
    	if(xbc.getXButton()) Drive.buttonXHeld();
    	if(xbc.getYButton()) Drive.buttonYHeld();
    	
    	if(xbc.getAButton()!=lastA&&xbc.getAButton()) Drive.buttonAPressed();
    	if(xbc.getBButton()!=lastB&&xbc.getBButton()) Drive.buttonBPressed();
    	if(xbc.getXButton()!=lastX&&xbc.getXButton()) Drive.buttonXPressed();
    	if(xbc.getYButton()!=lastY&&xbc.getYButton()) Drive.buttonYPressed();
    	if(xbc.getBumper(GenericHID.Hand.kRight)!=lastBumperR&&xbc.getBumper(GenericHID.Hand.kRight)) Drive.bumperRPressed();
    	if(xbc.getBumper(GenericHID.Hand.kLeft)!=lastBumperL&&xbc.getBumper(GenericHID.Hand.kLeft)) Drive.bumperLPressed();
    	if((xbc.getTriggerAxis(GenericHID.Hand.kRight)<.7)!=lastTriggerR&&(xbc.getTriggerAxis(GenericHID.Hand.kRight)<7)) Drive.triggerRPressed();
    	if((xbc.getTriggerAxis(GenericHID.Hand.kLeft)<.7)!=lastTriggerL&&(xbc.getTriggerAxis(GenericHID.Hand.kLeft)<7)) Drive.triggerLPressed();
    	
    	if(xbc.getAButton()!=lastA&&!xbc.getAButton()) Drive.buttonAReleased();
    	if(xbc.getBButton()!=lastB&&!xbc.getBButton()) Drive.buttonBReleased();
    	if(xbc.getXButton()!=lastX&&!xbc.getXButton()) Drive.buttonXReleased();
    	if(xbc.getYButton()!=lastY&&!xbc.getYButton()) Drive.buttonYReleased();
    	if(xbc.getBumper(GenericHID.Hand.kRight)!=lastBumperR&&!xbc.getBumper(GenericHID.Hand.kRight)) Drive.bumperRReleased();
    	if(xbc.getBumper(GenericHID.Hand.kLeft)!=lastBumperL&&!xbc.getBumper(GenericHID.Hand.kLeft)) Drive.bumperLReleased();
    	if((xbc.getTriggerAxis(GenericHID.Hand.kRight)<.7)!=lastTriggerR&&!(xbc.getTriggerAxis(GenericHID.Hand.kRight)<7)) Drive.triggerRReleased();
    	if((xbc.getTriggerAxis(GenericHID.Hand.kLeft)<.7)!=lastTriggerL&&!(xbc.getTriggerAxis(GenericHID.Hand.kLeft)<7)) Drive.triggerLReleased();
    	
    	lastA = xbc.getAButton();
    	lastB = xbc.getBButton();
    	lastX = xbc.getXButton();
    	lastY = xbc.getYButton();
    	lastBumperR = xbc.getBumper(GenericHID.Hand.kRight);
    	lastBumperL = xbc.getBumper(GenericHID.Hand.kLeft);
    	lastTriggerR = xbc.getTriggerAxis(GenericHID.Hand.kRight)<.7;
    	lastTriggerL = xbc.getTriggerAxis(GenericHID.Hand.kLeft)<.7;
    }
}

