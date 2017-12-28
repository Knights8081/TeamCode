package org.firstinspires.ftc.teamcode.testClasses.movement;

import android.os.SystemClock;

import org.firstinspires.ftc.teamcode.teamCode.HardwareNut;

/**
 * This class will test the movement of the robot. It will test 6 key movement features:
 *
 *   1) Moving forward
 *   2) Moving backward
 *   3) Turning left
 *   4) Turning right
 *   5) Strafing Left
 *   6) Strafing right
 *
 * @author Corbin Young
 */
public final class Move implements Runnable {

    private static final float POWERVALUE = 0.75f;
    private static final float STRAFEVALUE = 0.6f;

    //Value to set the power to 0 to stop the motor
    private static final float STOPVALUE = 0.0f;

    private HardwareNut robot;

    public Move(final HardwareNut robot) {
        this.robot = robot;
    }

    /**
     * Move the robot forward
     */
    private void forward() {
        robot.getRightDrive().setPower(-POWERVALUE);
        robot.getLeftDrive().setPower(-POWERVALUE);
        robot.getRightArm().setPower(-POWERVALUE);
        robot.getLeftArm().setPower(-POWERVALUE);
    }

    /**
     * Move the robot backward
     */
    private void backward() {
        robot.getRightDrive().setPower(POWERVALUE);
        robot.getLeftDrive().setPower(POWERVALUE);
        robot.getRightArm().setPower(POWERVALUE);
        robot.getLeftArm().setPower(POWERVALUE);
    }

    /**
     * Turn the robot to the left
     */
    private void turnLeft() {
        robot.getRightDrive().setPower(-POWERVALUE);
        robot.getLeftDrive().setPower(POWERVALUE);
        robot.getRightArm().setPower(-POWERVALUE);
        robot.getLeftArm().setPower(POWERVALUE);
    }

    /**
     * Turn the robot to the right
     */
    private void turnRight() {
        robot.getRightDrive().setPower(POWERVALUE);
        robot.getLeftDrive().setPower(-POWERVALUE);
        robot.getRightArm().setPower(POWERVALUE);
        robot.getLeftArm().setPower(-POWERVALUE);
    }

    /**
     * Strafe the robot to the left
     */
    private void strafeLeft() {
        robot.getRightDrive().setPower(-STRAFEVALUE);
        robot.getLeftDrive().setPower(STRAFEVALUE);
        robot.getRightArm().setPower(STRAFEVALUE);
        robot.getLeftArm().setPower(-STRAFEVALUE);
    }

    /**
     * Strafe the robot to the right
     */
    private void strafeRight() {
        robot.getRightDrive().setPower(POWERVALUE);
        robot.getLeftDrive().setPower(-POWERVALUE);
        robot.getRightArm().setPower(-POWERVALUE);
        robot.getLeftArm().setPower(POWERVALUE);
    }

    /**
     * Stops the robot so that it is no longer moving or turning
     */
    private void stop() {
        robot.getRightDrive().setPower(STOPVALUE);
        robot.getLeftDrive().setPower(STOPVALUE);
        robot.getRightArm().setPower(STOPVALUE);
        robot.getLeftArm().setPower(STOPVALUE);
    }

    /**
     * Test function to run through all of the movement tests
     */
    @Override
    public void run() {
        long shortWait = 7500;       //750ms pause
        long longWait = 1000;       //1s pause

        forward();
        SystemClock.sleep(shortWait);

        stop();
        SystemClock.sleep(longWait);

        backward();
        SystemClock.sleep(shortWait);

        stop();
        SystemClock.sleep(longWait);

        turnLeft();
        SystemClock.sleep(shortWait);

        stop();
        SystemClock.sleep(longWait);

        turnRight();
        SystemClock.sleep(shortWait);

        stop();
        SystemClock.sleep(longWait);

        strafeLeft();
        SystemClock.sleep(shortWait);

        stop();
        SystemClock.sleep(longWait);

        strafeRight();
        SystemClock.sleep(shortWait);

        stop();
        SystemClock.sleep(longWait);
    }
}
