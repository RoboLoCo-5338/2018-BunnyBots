package org.usfirst.frc.team5338.robot.commands;

import org.usfirst.frc.team5338.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Turn extends PIDCommand
{
	double angle, initalHeading, targetHeading, integral, previous_error = 0;

	public Turn(double input)
	{
		//Input in inches to travel
		super(0.05535, 0.0000083, 0);
		this.requires(Robot.drivetrain);
		this.requires(Robot.sensors);

		
		this.angle = input;
		getPIDController().setOutputRange(-0.6, 0.6);
		getPIDController().setInputRange(-180, 180);
		getPIDController().setContinuous();
		getPIDController().setP(0.05535);
		getPIDController().setI(0.0000083);
		getPIDController().setF(0.005);

		//LiveWindow.add(getPIDController());
		//SmartDashboard.putData(this);
		this.initalHeading = Robot.sensors.ahrs.getYaw();
		//SmartDashboard.putNumber("heading", this.initalHeading);
		// this.targetHeading = ((double) (-Robot.sensors.ahrs.getYaw()) + angle);
		// if (targetHeading < -180) {
	    // 	setSetpoint(180.0 + targetHeading % 180);
		// } else if (targetHeading > 180) {
	    // 	setSetpoint(-180 + targetHeading % 180);
		// } else {
	    // 	setSetpoint(targetHeading);
		// }

		getPIDController().setSetpoint(this.initalHeading + this.angle);

		//SmartDashboard.putNumber("error", getPIDController().getError());
		//SmartDashboard.putNumber("setpoint", getPIDController().getSetpoint());
		//SmartDashboard.putNumber("I", getPIDController().getI());
		
	
		this.setTimeout(15);
	}
	
	@Override
	protected void initialize()
	{
		super.initialize();
		Robot.sensors.resetEncoders();
	}
	@Override
	protected void execute()
	{
		//SmartDashboard.putNumber("CURRENT HEADING", Robot.sensors.ahrs.getYaw());
	}
	@Override
	protected boolean isFinished()
	{
		//return getPIDController().getError() < 0.01 || getPIDController().getError() > -0.01 ||

		return this.isTimedOut();
	}
	@Override
	protected void end()
	{
		Robot.drivetrain.drive(0.0, 0.0);
	}

	protected double returnPIDInput() {
		return -Robot.sensors.ahrs.getYaw();
		}
	
	@Override
	protected void usePIDOutput(double output) {
		Robot.drivetrain.drive(-output, output);
	}
}