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
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

import edu.wpi.first.wpilibj.command.Subsystem;


/**
 *
 */
public class Climb extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private final static SpeedController climb1 = RobotMap.climbClimb1;
    private final static SpeedController climb2 = RobotMap.climbClimb2;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS


    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }
    public static void climbStart(){
    	double mult = 1;//change as needed
    	climb1.set(mult);
    	climb2.set(mult);
    }
    public static void climbEnd(){
    	double mult = 0;
    	climb1.set(mult);
    	climb2.set(mult);
    }
}

