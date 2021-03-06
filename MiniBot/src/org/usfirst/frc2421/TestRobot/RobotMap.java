// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc2421.TestRobot;

import edu.wpi.first.wpilibj.DigitalInput;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

// END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static Spark driveDriveLeft1;
    public static Spark driveDriveLeft2;
    public static Spark driveDriveRight1;
    public static Spark driveDriveRight2;
    public static RobotDrive driveDriveSystem;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public static void init() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveDriveLeft1 = new Spark(0);
        //LiveWindow.addActuator("Drive", "DriveLeft1", (Talon) driveDriveLeft1);
        
        driveDriveLeft2 = new Spark(1);
        //LiveWindow.addActuator("Drive", "DriveLeft2", (Talon) driveDriveLeft2);
        
        driveDriveRight1 = new Spark(3);
        //LiveWindow.addActuator("Drive", "DriveRight1", (Talon) driveDriveRight1);
        
        driveDriveRight2 = new Spark(2);
        //LiveWindow.addActuator("Drive", "DriveRight2", (Talon) driveDriveRight2);
        
        driveDriveSystem = new RobotDrive(driveDriveLeft1, driveDriveLeft2,
              driveDriveRight1, driveDriveRight2);
        
        driveDriveSystem.setSafetyEnabled(true);
        driveDriveSystem.setExpiration(0.1);
        driveDriveSystem.setSensitivity(0.5);
        driveDriveSystem.setMaxOutput(1.0);
        

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }
}
