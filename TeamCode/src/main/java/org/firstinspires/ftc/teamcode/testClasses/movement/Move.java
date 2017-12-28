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
public final class Move {

    private static final float POWERVALUE = 0.75f;
    private static final float STRAFEVALUE = 0.6f;

    //Value to set the power to 0 to stop the motor
    private static final float STOPVALUE = 0.0f;

    private static final long SHORTWAIT = 300;      //300ms pause
    private static final long LONGWAIT = 425;      //425ms pause

    /**
     * Move the robot forward
     */
    private static void forward(final HardwareNut robot) {
        robot.getRightDrive().setPower(-POWERVALUE);
        robot.getLeftDrive().setPower(-POWERVALUE);
        robot.getRightArm().setPower(-POWERVALUE);
        robot.getLeftArm().setPower(-POWERVALUE);
    }

    /**
     * Move the robot backward
     */
    private static void backward(final HardwareNut robot) {
        robot.getRightDrive().setPower(POWERVALUE);
        robot.getLeftDrive().setPower(POWERVALUE);
        robot.getRightArm().setPower(POWERVALUE);
        robot.getLeftArm().setPower(POWERVALUE);
    }

    /**
     * Turn the robot to the left
     */
    private static void turnLeft(final HardwareNut robot) {
        robot.getRightDrive().setPower(-POWERVALUE);
        robot.getLeftDrive().setPower(POWERVALUE);
        robot.getRightArm().setPower(-POWERVALUE);
        robot.getLeftArm().setPower(POWERVALUE);
    }

    /**
     * Turn the robot to the right
     */
    private static void turnRight(final HardwareNut robot) {
        robot.getRightDrive().setPower(POWERVALUE);
        robot.getLeftDrive().setPower(-POWERVALUE);
        robot.getRightArm().setPower(POWERVALUE);
        robot.getLeftArm().setPower(-POWERVALUE);
    }

    /**
     * Strafe the robot to the left
     */
    private static void strafeLeft(final HardwareNut robot) {
        robot.getRightDrive().setPower(-STRAFEVALUE);
        robot.getLeftDrive().setPower(STRAFEVALUE);
        robot.getRightArm().setPower(STRAFEVALUE);
        robot.getLeftArm().setPower(-STRAFEVALUE);
    }

    /**
     * Strafe the robot to the right
     */
    private static void strafeRight(final HardwareNut robot) {
        robot.getRightDrive().setPower(POWERVALUE);
        robot.getLeftDrive().setPower(-POWERVALUE);
        robot.getRightArm().setPower(-POWERVALUE);
        robot.getLeftArm().setPower(POWERVALUE);
    }

    /**
     * Stops the robot so that it is no longer moving or turning
     */
    private static void stop(final HardwareNut robot) {
        robot.getRightDrive().setPower(STOPVALUE);
        robot.getLeftDrive().setPower(STOPVALUE);
        robot.getRightArm().setPower(STOPVALUE);
        robot.getLeftArm().setPower(STOPVALUE);
    }

    /**
     * Test function to run through all of the movement tests
     */
    public static void runMovementTest(final HardwareNut robot) {

        forward(robot);
        SystemClock.sleep(SHORTWAIT);

        stop(robot);
        SystemClock.sleep(LONGWAIT);

        backward(robot);
        SystemClock.sleep(SHORTWAIT);

        stop(robot);
        SystemClock.sleep(LONGWAIT);

        turnLeft(robot);
        SystemClock.sleep(SHORTWAIT);

        stop(robot);
        SystemClock.sleep(LONGWAIT);

        turnRight(robot);
        SystemClock.sleep(SHORTWAIT);

        stop(robot);
        SystemClock.sleep(LONGWAIT);

        strafeLeft(robot);
        SystemClock.sleep(SHORTWAIT);

        stop(robot);
        SystemClock.sleep(LONGWAIT);

        strafeRight(robot);
        SystemClock.sleep(SHORTWAIT);

        stop(robot);
        SystemClock.sleep(LONGWAIT);
    }
}
