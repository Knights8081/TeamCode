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
 * As I am not familiar with moving the robot, the functions are my best guess.
 * TODO - check the functions to make sure they are doing what I think they are
 *
 * @author Corbin Young
 */
public final class Move {

    /*
     * TODO - test for valid value to use for setting the power
     *
     * This class could also be used to test the values until a good one is found to use in
     *  the AutoStrafe class
     */
    private static final float POWERVALUE = 0.75f;
    private static final float STRAFEVALUE = 0.6f;

    //Value to set the power to 0 to stop the motor
    private static final float STOPVALUE = 0.0f;

    /**
     * Move the robot forward
     *
     * @param robot - reference to the robot
     */
    private static void forward(final HardwareNut robot) {
        robot.getRightDrive().setPower(-POWERVALUE);
        robot.getLeftDrive().setPower(-POWERVALUE);
        robot.getRightArm().setPower(-POWERVALUE);
        robot.getLeftArm().setPower(-POWERVALUE);
    }

    /**
     * Move the robot backward
     *
     * @param robot - reference to the robot
     */
    private static void backward(final HardwareNut robot) {
        robot.getRightDrive().setPower(POWERVALUE);
        robot.getLeftDrive().setPower(POWERVALUE);
        robot.getRightArm().setPower(POWERVALUE);
        robot.getLeftArm().setPower(POWERVALUE);
    }

    /**
     * Turn the robot to the left
     *
     * @param robot - reference to the robot
     */
    private static void turnLeft(final HardwareNut robot) {
        robot.getRightDrive().setPower(-POWERVALUE);
        robot.getLeftDrive().setPower(POWERVALUE);
        robot.getRightArm().setPower(-POWERVALUE);
        robot.getLeftArm().setPower(POWERVALUE);
    }

    /**
     * Turn the robot to the right
     *
     * @param robot - reference to the robot
     */
    private static void turnRight(final HardwareNut robot) {
        robot.getRightDrive().setPower(POWERVALUE);
        robot.getLeftDrive().setPower(-POWERVALUE);
        robot.getRightArm().setPower(POWERVALUE);
        robot.getLeftArm().setPower(-POWERVALUE);
    }

    /**
     * Strafe the robot to the left
     *
     * @param robot - reference to the robot
     */
    private static void strafeLeft(final HardwareNut robot) {
        robot.getRightDrive().setPower(-STRAFEVALUE);
        robot.getLeftDrive().setPower(STRAFEVALUE);
        robot.getRightArm().setPower(STRAFEVALUE);
        robot.getLeftArm().setPower(-STRAFEVALUE);
    }

    /**
     * Strafe the robot to the right
     *
     * @param robot - reference to the robot
     */
    private static void strafeRight(final HardwareNut robot) {
        robot.getRightDrive().setPower(POWERVALUE);
        robot.getLeftDrive().setPower(-POWERVALUE);
        robot.getRightArm().setPower(-POWERVALUE);
        robot.getLeftArm().setPower(POWERVALUE);
    }

    /**
     * Stops the robot so that it is no longer moving or turning
     *
     * @param robot - reference to the robot
     */
    private static void stop(final HardwareNut robot) {
        robot.getRightDrive().setPower(STOPVALUE);
        robot.getLeftDrive().setPower(STOPVALUE);
        robot.getRightArm().setPower(STOPVALUE);
        robot.getLeftArm().setPower(STOPVALUE);
    }

    /**
     * Test function to run through all of the movement tests
     *
     * @param robot - reference to the robot
     */
    public static void runMovementTest(final HardwareNut robot) {
        long shortWait = 750;       //750ms pause
        long longWait = 2000;       //2s pause

        forward(robot);
        SystemClock.sleep(shortWait);

        stop(robot);
        SystemClock.sleep(longWait);

        backward(robot);
        SystemClock.sleep(shortWait);

        stop(robot);
        SystemClock.sleep(longWait);

        turnLeft(robot);
        SystemClock.sleep(shortWait);

        stop(robot);
        SystemClock.sleep(longWait);

        turnRight(robot);
        SystemClock.sleep(shortWait);

        stop(robot);
        SystemClock.sleep(longWait);

        strafeLeft(robot);
        SystemClock.sleep(shortWait);

        stop(robot);
        SystemClock.sleep(longWait);

        strafeRight(robot);
        SystemClock.sleep(shortWait);

        stop(robot);
        SystemClock.sleep(longWait);
    }
}
