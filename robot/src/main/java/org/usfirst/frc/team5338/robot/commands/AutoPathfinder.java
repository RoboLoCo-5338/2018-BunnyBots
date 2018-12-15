package org.usfirst.frc.team5338.robot.commands;

import org.usfirst.frc.team5338.robot.Robot;
import org.usfirst.frc.team5338.robot.subsystems.DriveTrain;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.Trajectory.FitMethod;
import jaci.pathfinder.followers.EncoderFollower;
import jaci.pathfinder.modifiers.TankModifier;

public class AutoPathfinder extends Command
{

    Waypoint[] points =  new Waypoint[] {
        new Waypoint(0, 0, 0),
        new Waypoint(-2, -2, Pathfinder.d2r(-45))
    };

    Trajectory trajectory = null;
    TankModifier modifier = null;
    EncoderFollower left = null;
    EncoderFollower right = null;

	public AutoPathfinder()
	{
        this.requires(Robot.drivetrain);
        this.requires(Robot.sensors);

        Trajectory.Config config = new Trajectory.Config(FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 5.107479538, 3.530394, Double.MAX_VALUE);
        trajectory = Pathfinder.generate(points, config);
        modifier = new TankModifier(trajectory).modify(0.61);
        left = new EncoderFollower(modifier.getLeftTrajectory());
        right = new EncoderFollower(modifier.getRightTrajectory());
        left.configureEncoder(Math.abs(Robot.drivetrain.LEFT_1.getSensorCollection().getQuadraturePosition()), 4096, 0.15);
        right.configureEncoder(Math.abs(Robot.drivetrain.RIGHT_2.getSensorCollection().getQuadraturePosition()), 4096, 0.15);
        left.configurePIDVA(1.00, 0.00, 0.00, 1.0 / 5.107479538, 0);
        right.configurePIDVA(1.00, 0.00, 0.00, 1.0 / 5.107479538, 0);

        this.setTimeout(1000);
    }
	@Override
	protected void end()
	{

    }
	@Override
	protected void execute()
	{
        modifier = new TankModifier(trajectory).modify(0.61);
        left = new EncoderFollower(modifier.getLeftTrajectory());
        right = new EncoderFollower(modifier.getRightTrajectory());
        left.configureEncoder(Math.abs(Robot.drivetrain.LEFT_1.getSensorCollection().getQuadraturePosition()), 4096, 0.15);
        right.configureEncoder(Math.abs(Robot.drivetrain.RIGHT_2.getSensorCollection().getQuadraturePosition()), 4096, 0.15);
        left.configurePIDVA(100.00, 0.00, 0.00, 1.0 / 5.107479538, 0);
        right.configurePIDVA(100.00, 0.00, 0.00, 1.0 / 5.107479538, 0);

        int leftEncoder = Math.abs(Robot.drivetrain.LEFT_1.getSensorCollection().getQuadraturePosition());
        int rightEncoder = Math.abs(Robot.drivetrain.RIGHT_2.getSensorCollection().getQuadraturePosition());
        double l = left.calculate(leftEncoder);
        double r = right.calculate(rightEncoder);
        double gyroHeading = Robot.sensors.ahrs.getAngle();
        double desiredHeading = Pathfinder.r2d(left.getHeading());
        double angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
        double turn = 0.8 * (-1.0/80.0) * angleDifference;
        SmartDashboard.putNumber("Right Profile", (r + turn));
        SmartDashboard.putNumber("Left Profile", (l - turn));
        SmartDashboard.putNumber("encoder Left ", leftEncoder);
        SmartDashboard.putNumber("encoder Right", rightEncoder);
        SmartDashboard.putNumber("Turn", turn);
        SmartDashboard.putNumber("Desired Heading", desiredHeading);
        SmartDashboard.putNumber("Left Drive", -(r + turn));
        SmartDashboard.putNumber("Right Drive", -(l - turn));

        // SmartDashboard.putNumber("turn", turn);
    
        Robot.drivetrain.drive(l - turn, r + turn);
    }
	@Override
	protected boolean isFinished()
	{
		return this.isTimedOut();
	}
}
