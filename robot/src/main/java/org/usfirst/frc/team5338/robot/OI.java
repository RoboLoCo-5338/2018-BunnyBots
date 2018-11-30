//Package for all of our code.
package org.usfirst.frc.team5338.robot;

//Import of all essential wpilib classes.
import edu.wpi.first.wpilibj.Joystick;

//Main class that handles Operator Input.
public class OI
{
	// Enum that represents all possible buttons in use.
	public enum Button // Shoot
	{
		STRAIGHT, SHIFT_DOWN, INTAKE, OUTTAKE, CLOSE, OPEN, EXTEND, RETRACT, FLOOR, SWITCH, SCALE, SPIN_UP, SHOOT, SENS
	}
	
	// Private method that returns a deadzone-adjusted value for a joystick value
	// input.
	private static double joystickDeadZone(final double value)
	{
		// if(value > 0.075)
		// {
		// 	return (value - 0.075) / 0.925;
		// }
		// else if(value < -0.075)
		// {
		// 	return (value + 0.075) / 0.925;
		// }
		return value;
	}
	
	// Creates private joysticks objects for use.
	private final Joystick leftController = new Joystick(0);
	private final Joystick rightController = new Joystick(1);
	
	// Public method that returns the state of a particular button based on the
	// Button enum.
	public boolean get(final Button button)
	{
		// TODO CHECK CONTROL SCHEME WORKS
		switch(button)
		{
			case CLOSE:
				return this.leftController.getRawButton(9) || this.leftController.getRawButton(1);
			case OPEN:
				return this.leftController.getRawButton(1);
			case INTAKE:
				return this.leftController.getRawButton(5);
			case OUTTAKE:
				return this.leftController.getRawButton(6);
			case SPIN_UP:
				return this.leftController.getRawAxis(2) > 0.1 ? true : false;
			case SHOOT:
				return this.leftController.getRawAxis(3) > 0.1 ? true : false;
			case EXTEND:
				return this.rightController.getRawButton(9);
			case RETRACT:
				return this.rightController.getRawButton(10);
			case STRAIGHT:
				return this.leftController.getRawButton(2);
			case SHIFT_DOWN:
				return this.rightController.getRawButton(8);
			case FLOOR:
				return this.leftController.getPOV(0) == 180 ? true : false;
			case SWITCH:
				return this.leftController.getPOV(0) == 90 || this.leftController.getPOV(0) == 270 ? true : false;
			case SCALE:
				return this.leftController.getPOV(0) == 0 ? true : false;
			case SENS:
				return this.leftController.getRawButtonReleased(10);
			default:
				return false;
		}
	}
	// Public method that returns the left joystick's deadzone-adjusted values
	public double getLeftJoystick(final char input)
	{
		switch(input)
		{
			case 'X': // Gets deadzone corrected x-axis position
				return OI.joystickDeadZone(this.leftController.getRawAxis(0));
			case 'Y': // Gets deadzone corrected y-axis position
				return -OI.joystickDeadZone(this.leftController.getRawAxis(1));
			case 'Z': // Gets deadzone corrected z-axis (rotation) position
				return 0.0;
			default: // Returns 0.0 is argument is unknown
				return 0.0;
		}
	}
	// Public method that returns the right joystick's deadzone-adjusted values
	public double getRightJoystick(final char input)
	{
		switch(input)
		{
			case 'X': // Gets deadzone corrected x-axis position
				return OI.joystickDeadZone(this.leftController.getRawAxis(4));
			case 'Y': // Gets deadzone corrected y-axis position
				return -OI.joystickDeadZone(this.leftController.getRawAxis(5));
			case 'Z': // Gets deadzone corrected z-axis (rotation) position
				return 0.0;
			default: // Returns 0.0 is argument is unknown
				return 0.0;
		}
	}
}
