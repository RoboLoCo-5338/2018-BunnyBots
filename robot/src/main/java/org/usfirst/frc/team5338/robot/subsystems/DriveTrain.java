package org.usfirst.frc.team5338.robot.subsystems;

import org.usfirst.frc.team5338.robot.OI;
import org.usfirst.frc.team5338.robot.Robot;
import org.usfirst.frc.team5338.robot.commands.JoystickControl;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.SensorCollection;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Class in which Robot calls to perform all functions: specifically Drive Train
public class DriveTrain extends Subsystem
{
	// Field variables that we will use
	// Talons: motor controllers that we use on the robot
	private final WPI_TalonSRX LEFT_1 = new WPI_TalonSRX(1);
	public final WPI_TalonSRX LEFT_2 = new WPI_TalonSRX(2);
	public final WPI_TalonSRX RIGHT_1 = new WPI_TalonSRX(3);
	public final WPI_TalonSRX RIGHT_2 = new WPI_TalonSRX(4);
	// Contol-group objects used to move the left drive drive and right drive motors
	// in sync (two drive motors per side)
	private final SpeedControllerGroup LEFT_SIDE = new SpeedControllerGroup(this.LEFT_1, this.LEFT_2);
	private final SpeedControllerGroup RIGHT_SIDE = new SpeedControllerGroup(this.RIGHT_1, this.RIGHT_2);
	// Creates a drive object that will define how the left and right motor sets are
	// configured (currently as an arcade drive)
	private final DifferentialDrive DRIVE = new DifferentialDrive(this.LEFT_SIDE, this.RIGHT_SIDE);
	// Objects that control the shift and compressor mechanism
	private final Compressor COMPRESSOR = new Compressor(8);
	private final DoubleSolenoid SHIFTER = new DoubleSolenoid(8, 1, 2);
	private boolean straight;
	private boolean sensChange = false;
	private double speed = 3.0/5.5;;
	private double maxLVelocity = 0.0;
	private double maxRVelocity = 0.0;
	private double accelRight = 0.0;
	private double accelLeft = 0.0;
	private double maxLAccel = 0.0;
	private double maxRAccel = 0.0;
	
	// Use constructor for any pre-start initialization
	public DriveTrain()
	{
		super();
		this.COMPRESSOR.setClosedLoopControl(true);
		this.COMPRESSOR.start();
		this.SHIFTER.set(DoubleSolenoid.Value.kForward);
		this.straight = false;
		for(final WPI_TalonSRX talon : new WPI_TalonSRX[] {this.LEFT_1, this.LEFT_2, this.RIGHT_1, this.RIGHT_2})
		{
			DriveTrain.configureTalon(talon);
		}
		this.SHIFTER.set(DoubleSolenoid.Value.kForward);
	}
	private static void configureTalon(final WPI_TalonSRX talon)
	{
		talon.configPeakCurrentLimit(100, 0);
		talon.configPeakCurrentDuration(3, 0);
		talon.configContinuousCurrentLimit(80, 0);
		talon.enableCurrentLimit(true);
		talon.configNeutralDeadband(0.001, 0);
		talon.setStatusFramePeriod(StatusFrame.Status_1_General, 5, 0);
		talon.setControlFramePeriod(ControlFrame.Control_3_General, 5);
	}
	public void shift(final boolean state)
	{
		if(state)
		{
			this.SHIFTER.set(DoubleSolenoid.Value.kReverse);
			this.straight = true;
		}
		else
		{
			this.SHIFTER.set(DoubleSolenoid.Value.kForward);
			this.straight = false;
		}
	}
	public SensorCollection[] getEncoders()
	{
		return new SensorCollection[] {this.LEFT_1.getSensorCollection(), this.RIGHT_2.getSensorCollection()};
	}
	// Runs the drive in tank mode
	public void drive(final double left, final double right)
	{
		this.DRIVE.tankDrive(left, right);
	}
	// Actual drive method called in Robot class
	public void drive(final OI oi)
	{
		
		if(oi.get(OI.Button.STRAIGHT))
		{
			
			this.DRIVE.tankDrive((oi.getLeftJoystick('Y') * speed), (oi.getLeftJoystick('Y') * speed), false);
		}
		else
		{
			this.DRIVE.tankDrive((oi.getLeftJoystick('Y') * speed), (oi.getRightJoystick('Y') * speed), false);
		}

		if(oi.get(OI.Button.SENS)) { 
			this.sensChange = !this.sensChange;
		}

		if(sensChange) {
			speed = 0.25;
		} else {
			speed = 1;
		}

		SmartDashboard.putBoolean("sens", !sensChange);

		final double[] velocity = Robot.sensors.velocity();

		final double[] prevVelocity = Robot.sensors.prevVelocity();

		final double[] time = Robot.sensors.time();

		accelLeft = (velocity[0] - prevVelocity[0])/(time[1] - time[0]);
		accelRight = (velocity[1] - prevVelocity[1])/(time[1] - time[0]);

		if(Math.abs(velocity[0]) > maxLVelocity) {
			
		 	maxLVelocity = Math.abs(velocity[0]);
		}

		 if(Math.abs(velocity[1]) > maxRVelocity) {
		 	maxRVelocity = Math.abs(velocity[1]);
		 }

		 if(Math.abs(accelLeft) > maxLAccel) { 
			 maxLAccel = Math.abs(accelLeft);
		 }

		 if(accelRight > maxRAccel) {
			 maxRAccel = Math.abs(accelRight);
		 }

		 SmartDashboard.putNumber("MaxVel", 0.5 * (maxLVelocity + maxRVelocity));
		 SmartDashboard.putNumber("Max Accel", 0.5 * (Math.abs(maxLAccel) + Math.abs(maxRAccel)));

		// if(Robot.sensors.accel() > maxAccel) {
		// 	maxAccel = Robot.sensors.accel();
		// }


		// SmartDashboard.putNumber("pos", 0.5 * (1.0/4096.0) * (6.0/Math.PI) * (Math.abs(this.getEncoders()[0].getQuadraturePosition()) + Math.abs(this.getEncoders()[1].getQuadraturePosition())));
		// SmartDashboard.putNumber("leftPos", this.getEncoders()[0].getQuadraturePosition());
		// SmartDashboard.putNumber("rightPos", this.getEncoders()[1].getQuadraturePosition());

		// if(Robot.oi.get(OI.Button.SHIFT_UP))
		// {
		// // If the user has allowed the gear to shift up, then change the solenoid to
		// // allow for faster movement
		// this.SHIFTER.set(DoubleSolenoid.Value.kReverse);
		// this.straight = true;
		// }
		// else if(Robot.oi.get(OI.Button.SHIFT_DOWN))
		// {
		// // If user forces the gear to shift down, only do so if power is < 25%
		// if((Math.abs(this.LEFT_SIDE.get()) <= 0.25) &&
		// (Math.abs(this.RIGHT_SIDE.get()) <= 0.25))
		// {
		// this.SHIFTER.set(DoubleSolenoid.Value.kForward);
		// this.straight = false;
		// }
		// }
		/**
		 * IMPORTANT: Due to motor mirroring Forward: Left = +, Right = - Backward: Left
		 * = -, Right = +
		 */
		SmartDashboard.putBoolean("Shift Status", this.straight);
		SmartDashboard.putData("Drivetrain Status", this.DRIVE);
	}
	// Default drive command will be a general tank drive instead of arcade drive
	@Override
	public void initDefaultCommand()
	{
		this.setDefaultCommand(new JoystickControl());
	}
}
