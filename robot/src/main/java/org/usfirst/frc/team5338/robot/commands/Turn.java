package org.usfirst.frc.team5338.robot.commands;

import org.usfirst.frc.team5338.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class Turn extends Command
{
	double error, initalHeading, targetHeading;
	double P, I, D;
	
	public Turn(final double input)
	{
		// Input in inches to travel
		super();
		this.requires(Robot.drivetrain);
		this.requires(Robot.sensors);
		this.initalHeading = Robot.sensors.ahrs.getYaw();
		this.targetHeading = this.initalHeading + input;
		this.error = this.targetHeading - this.initalHeading;
		this.setTimeout((2 * Math.abs(this.error)) / 90.0);
	}
	public void PID() {

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
		if(this.error > Robot.sensors.ahrs.getYaw())
		{
			Robot.drivetrain.drive(0.70, -0.70);
		}
		else
		{
			Robot.drivetrain.drive(-0.70, 0.70);
		}
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
}