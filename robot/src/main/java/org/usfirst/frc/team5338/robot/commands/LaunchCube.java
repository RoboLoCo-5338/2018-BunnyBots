package org.usfirst.frc.team5338.robot.commands;

import org.usfirst.frc.team5338.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

public class LaunchCube extends Command
{
	public LaunchCube()
	{
		super();
		this.requires(Robot.claw);
		this.setTimeout(0.50);
	}
	@Override
	protected void execute()
	{
		Robot.claw.shoot();
	}
	@Override
	protected boolean isFinished()
	{
		// TODO CHECK TIMEOUT IS ENOUGH AND FUNCTIONALITY
		return this.isTimedOut();
	}
	@Override
	protected void end()
	{
		Robot.claw.resetShooter();
	}
}