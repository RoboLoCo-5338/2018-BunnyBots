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
        new Waypoint(10, 0, 0)
    };

    Trajectory trajectory = null;
    TankModifier modifier = null;
    EncoderFollower left = null;
    EncoderFollower right = null;
    // File myFile = null;
    int leftEncoder, rightEncoder;
    double gyroHeading, l, r, desiredHeading, angleDifference, turn;

	public AutoPathfinder()
	{
        this.requires(Robot.drivetrain);
        this.requires(Robot.sensors);

        Trajectory.Config config = new Trajectory.Config(FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH, 0.05, 5.107479538, 3.530394, Double.POSITIVE_INFINITY);
        trajectory = Pathfinder.generate(points, config);
        modifier = new TankModifier(trajectory).modify(0.61);

        // myFile = new File("myfile.csv");
        // Pathfinder.writeToCSV(myFile, trajectory);

        this.setTimeout(1000);
    }
	@Override
	protected void end()
	{

    }
	@Override
	protected void execute()
	{
       
        left = new EncoderFollower(modifier.getLeftTrajectory());
        right = new EncoderFollower(modifier.getRightTrajectory());
        left.configureEncoder(Math.abs(Robot.drivetrain.LEFT_1.getSensorCollection().getQuadraturePosition()), 4096, 0.15);
        right.configureEncoder(Math.abs(Robot.drivetrain.RIGHT_2.getSensorCollection().getQuadraturePosition()), 4096, 0.15);
        left.configurePIDVA(1.2, 0.00, 0.00, 1.0 / 5.107479538, 0);
        right.configurePIDVA(1.2, 0.00, 0.00, 1.0 / 5.107479538, 0);

        leftEncoder = Math.abs(Robot.drivetrain.LEFT_1.getSensorCollection().getQuadraturePosition());
        rightEncoder = Math.abs(Robot.drivetrain.RIGHT_2.getSensorCollection().getQuadraturePosition());
        l = left.calculate(leftEncoder);
        r = right.calculate(rightEncoder);
        SmartDashboard.putNumber("left_calc", l);
        SmartDashboard.putNumber("right_calc", r);
        gyroHeading = Robot.sensors.ahrs.getAngle();
        SmartDashboard.putNumber("heading", gyroHeading);
        desiredHeading = Pathfinder.r2d(left.getHeading());
        angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
        turn = 0.8 * (-1.0/80.0) * angleDifference;

        // if(l - turn > 0.55) {
        //     l = 0.5(l);
        //     turn = 0.5(turn);
        // }
    
        Robot.drivetrain.drive((l - turn), (r + turn));
    }
	@Override
	protected boolean isFinished()
	{
		return this.isTimedOut();
	}
}
