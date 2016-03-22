// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc841.Reptar.commands;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc841.Reptar.Robot;

/**
 * Climbs up the poll
 */
public class ClimbUp extends Command {
	private double speed = 0.3;
	private boolean isForward = true;
	private double Kp = 0.01;
	private double distance = 113; // climb in inches 
	
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS
 
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_DECLARATIONS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
    public ClimbUp() {

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=VARIABLE_SETTING
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
        requires(Robot.drivetrain);

        // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.drivetrain.initEncoder();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {

		double left, right, errorsum;

		// sample distance traveled
		left = Robot.drivetrain.getLeftEncoderDistance();
		right = Robot.drivetrain.getRightEncoderDistance();

		// P control loop to keep right side the same as left side; therefore,
		// making the robot go straight.
		errorsum = (left + right) * this.Kp;
		
		// limit correction for protection
		if (errorsum > 0.1) {
			errorsum = 0.1;
		} else if (errorsum < -0.1) {
			errorsum = -0.1;
		}

		// Update speed
		if (!this.isForward) {
			Robot.drivetrain.SetLeftRight(-this.speed, this.speed + errorsum);
		} else {
			Robot.drivetrain.SetLeftRight(this.speed, -this.speed + errorsum);
		}
		//System.out.println("errorsum: "+ errorsum);
	

    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Math.abs(Robot.drivetrain.getLeftEncoderDistance()) > this.distance;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.SetLeftRight(0, 0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
