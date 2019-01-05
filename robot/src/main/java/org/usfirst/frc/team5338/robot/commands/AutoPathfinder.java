package org.usfirst.frc.team5338.robot.commands;

import java.io.File;

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

    Trajectory trajectory = null;
    TankModifier modifier = null;
    EncoderFollower left = null;
    EncoderFollower right = null;
    // File myFile = null;
    int leftEncoder, rightEncoder;
    File myFileLeft = new File("move_forward_left_Jaci.csv");
    File myFileRight = new File("move_forward_right_Jaci.csv");
    double gyroHeading, l, r, desiredHeading, angleDifference, turn;

	public AutoPathfinder()
	{
        this.requires(Robot.drivetrain);
        this.requires(Robot.sensors);

        
        Trajectory leftTrajectory = Pathfinder.readFromCSV(myFileLeft);

        
        Trajectory rightTrajectory = Pathfinder.readFromCSV(myFileRight);
        

        left = new EncoderFollower(leftTrajectory);
        right = new EncoderFollower(rightTrajectory);

        left.configureEncoder(Math.abs(Robot.drivetrain.LEFT_1.getSensorCollection().getQuadraturePosition()), 4096, 0.15);
        right.configureEncoder(Math.abs(Robot.drivetrain.RIGHT_2.getSensorCollection().getQuadraturePosition()), 4096, 0.15);
        left.configurePIDVA(1.2, 0.00, 0.00, 1.0 / 5.107479538, 0);
        right.configurePIDVA(1.2, 0.00, 0.00, 1.0 / 5.107479538, 0);

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

        leftEncoder = Robot.drivetrain.LEFT_1.getSensorCollection().getQuadraturePosition();
        SmartDashboard.putNumber("left_encoder", leftEncoder);
        rightEncoder = Robot.drivetrain.RIGHT_2.getSensorCollection().getQuadraturePosition();
        SmartDashboard.putNumber("right endocer", rightEncoder);
        l = left.calculate(leftEncoder);
        r = right.calculate(rightEncoder);
        SmartDashboard.putNumber("left_calc", l);
        SmartDashboard.putNumber("right_calc", r);

        gyroHeading = Robot.sensors.ahrs.getYaw();
        SmartDashboard.putNumber("heading", gyroHeading);
        desiredHeading = Pathfinder.r2d(left.getHeading());
        angleDifference = Pathfinder.boundHalfDegrees(desiredHeading - gyroHeading);
        turn = 0.8 * (-1.0/80.0) * angleDifference;

        Robot.drivetrain.drive(l + turn, r - turn);

        // if(l - turn > 0.55) {
        //     l = 0.5(l);
        //     turn = 0.5(turn);
        // }
    
    }
	@Override
	protected boolean isFinished()
	{
		return this.isTimedOut();
	}
}
