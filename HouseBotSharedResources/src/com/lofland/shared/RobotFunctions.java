package com.lofland.shared;

//A collection of classes for the robot

import lejos.nxt.LCD;
import lejos.nxt.Motor;
import lejos.nxt.MotorPort;
import lejos.nxt.NXTMotor;
import lejos.robotics.navigation.DifferentialPilot;
import lejos.util.Delay;

public class RobotFunctions {

	public static boolean rotateByAbsoluteValue(DifferentialPilot pilot, int rotationValue) {
		float moveSpeed = 25; // Going too fast makes this unreliable
		moveSpeed = (float) (moveSpeed / 100.0 * pilot.getMaxTravelSpeed());
		pilot.setTravelSpeed(moveSpeed);
		pilot.setRotateSpeed(moveSpeed);
		pilot.rotate(rotationValue);
		/*
		 * returnInformation = "ROTATED:" + headingDifference; It may work later to check how far it rotated by the compass and return that, kind of to verify if it worked.
		 */
		return true;
	}

	public static int GetHeadingError(int initialHeading, int finalHeading)
	// http://stackoverflow.com/questions/5024375/getting-the-difference-between-two-headings
	{
		if (initialHeading > 360 || initialHeading < 0 || finalHeading > 360 || finalHeading < 0) {
			return 360;
		}

		int diff = finalHeading - initialHeading;
		int absDiff = Math.abs(diff);

		if (absDiff <= 180) {
			return absDiff == 180 ? absDiff : diff;
		}

		else if (finalHeading > initialHeading) {
			return absDiff - 360;
		}

		else {
			return 360 - absDiff;
		}
	}

	/*
	 * A function to stop the robot, since it takes multiple commands to do it properly.
	 */
	public static void robotStop(DifferentialPilot pilot) {
		pilot.stop();
		// Always float the motors so they don't whine and burn up battery
		// power!
		Motor.B.flt(true);
		Motor.C.flt(true);
	}

	/*
	 * A function to convert the heading from the compass driver to a standard format I like to use where it is always positive, with the only "wrap" happening at 359 to 0 This
	 * also takes a float and returns a int
	 */

	public static int convertHeading(float theHeading) {
		// Correct for when signs are reversed
		if (theHeading < 0)
			theHeading += 2 * Math.PI;
		// Correct for wrap due to addition of declination
		if (theHeading > 2 * Math.PI)
			theHeading -= 2 * Math.PI;
		// Convert radians to degrees
		int finalHeading = (int) (theHeading * 180 / Math.PI);
		return finalHeading;
	}

	public static int cameraPositionSet() {
		/*
		 * Set up camera arm stops and angle settings: See RotateStall.java for camera rotation testing
		 */
		// Constants, consider where these should live?
		Motor.A.setSpeed(40); // Set regulated motor power: It should move very slowly (in regulated mode)
		int camMotorStallDelay = 250; // How long to wait for stall on camera motor
		int postStallRestTime = 500; // How long to rest after stalling camera motor
		int camearMotorPower = 60; // Power for unregulated motor stall testing
		// 50 worked until I got 3 US sensors, but now it cannot get over at 50

		LCD.clear(0);
		LCD.clear(1);
		LCD.drawString("Setting camera", 0, 0);
		LCD.drawString("position . . .", 0, 1);

		// Reset position by moving fully forward. Note motor is reversed.
		rotateToStop(false, camMotorStallDelay, postStallRestTime, camearMotorPower);

		// Find "reverse" rotation limit and make that 0
		rotateToStop(true, camMotorStallDelay, postStallRestTime, camearMotorPower);

		// This sets the current position as "0"
		Motor.A.resetTachoCount(); // Make this position "0", note this also "takes over" the motor and you need to let it go

		// Find "forward" rotation limit
		rotateToStop(false, camMotorStallDelay, postStallRestTime, camearMotorPower);

		//forwardCameraPosition = Motor.A.getTachoCount(); // This is the forward position limit

		/*
		 * Move to calculated "default" position by percentage Moved to connect function int cameraPosition = (int) (forwardCameraPosition * (defaultCameraAngle / 100.0));
		 * Motor.A.rotateTo(cameraPosition);
		 */
		LCD.clear(0);
		LCD.clear(1);
		
		return Motor.A.getTachoCount(); // This is the forward position limit
	}
	
	public static void rotateToStop(boolean reverse, int camMotorStallDelay, int postStallRestTime,
			int camearMotorPower) {
		/*
		 * This will rotate Motor on port A to stall Direction, stall delay, rest and power are passed in as arguments It may make sense to set these outside of either this method
		 * or the calling program? Ultimately they are a function of the robot's design. The robot design could be a class or config file itself
		 */
		Motor.A.suspendRegulation(); // "Release" the motor so that the
										// unregulated method can still access
										// it, in case it was locked by some
										// previous command

		// Create an unregulated motor instance
		int previousPosition, newPosition; // Create variables for position
											// sensing
		NXTMotor cameraMotor = new NXTMotor(MotorPort.A);
		cameraMotor.setPower(camearMotorPower);
		// "reverse" is dependent on motor orientation
		// This allows you to hide that from the method user
		if (reverse)
			cameraMotor.backward();
		else
			cameraMotor.forward();
		do {
			previousPosition = cameraMotor.getTachoCount();
			Delay.msDelay(camMotorStallDelay); // Give it time to stall and
												// don't "hog" the CPU
			newPosition = cameraMotor.getTachoCount();
		} while (previousPosition != newPosition);
		cameraMotor.flt();
		Delay.msDelay(postStallRestTime);
		return;
	}

}
