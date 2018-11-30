package org.usfirst.frc.team5338.robot.commands;

import org.usfirst.frc.team5338.robot.Robot;

import edu.wpi.first.wpilibj.command.PIDCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Turn extends PIDCommand
{
	double angle, initalHeading, targetHeading, integral, previous_error = 0;
	public Turn(final double input)
	{
		//Input in inches to travel
		super(1.00, 1.00, 1.00);
		this.requires(Robot.drivetrain);
		this.requires(Robot.sensors);

		
		this.angle = input;
		getPIDController().setOutputRange(-0.425, 0.425);
		getPIDController().setInputRange(-180, 180);
		getPIDController().setContinuous();

		SmartDashboard.putData(getPIDController());

		this.initalHeading = Robot.sensors.ahrs.getYaw();
		this.targetHeading = ((double) (-this.initalHeading) + input);
		if (targetHeading < -180) {
			setSetpoint(180.0 + targetHeading % 180);
		} else if (targetHeading > 180) {
			setSetpoint(-180 + targetHeading % 180);
		} else {
			setSetpoint(targetHeading);
		}
	
		this.setTimeout((3 * Math.abs(this.angle)) / 90.0);
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
		SmartDashboard.putNumber("CURRENT HEADING", Robot.sensors.ahrs.getYaw());
	}
	@Override
	protected boolean isFinished()
	{
		return (Math.abs(Robot.sensors.ahrs.getYaw() - this.targetHeading) < 2) || this.isTimedOut();
	}
	@Override
	protected void end()
	{
		Robot.drivetrain.drive(0.0, 0.0);
	}

	protected double returnPIDInput() {
		return -Robot.sensors.ahrs.getYaw();
		}
	
	protected void usePIDOutput(double output) {
		Robot.drivetrain.drive(-output, output);
	}
}