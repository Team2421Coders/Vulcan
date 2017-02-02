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

import org.usfirst.frc2421.Vulcan.RobotMap;
import org.usfirst.frc2421.Vulcan.commands.*;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Drive extends Subsystem {
	static double speedMult = 0.5;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final static CANTalon driveLeft1 = RobotMap.driveDriveLeft1;
    private final static CANTalon driveLeft2 = RobotMap.driveDriveLeft2;
    private final static CANTalon driveRight1 = RobotMap.driveDriveRight1;
    private final static CANTalon driveRight2 = RobotMap.driveDriveRight2;
    private final RobotDrive driveSystem = RobotMap.driveDriveSystem;

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
}

