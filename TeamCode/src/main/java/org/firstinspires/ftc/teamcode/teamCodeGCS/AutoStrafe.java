package org.firstinspires.ftc.teamcode.teamCodeGCS;

import org.firstinspires.ftc.teamcode.teamCode.HardwareNut;

/**
 * This class handles automatic strafing for the robot
 *
 * @author Luke Frazer
 */
public class AutoStrafe {

    //TODO - Test for value to use for setting power
    private static final float powerValue = 0.0f;

    /**
     * This method handles strafing left
     *
     * @param robot - reference to the actual robot
     */
    public static void left(final HardwareNut robot) {

        //Sequence: rD -, lD +, rA +, lA -
        robot.rightDrive.setPower(powerValue);
        robot.leftDrive.setPower(powerValue);
        robot.rightArm.setPower(powerValue);
        robot.leftArm.setPower(powerValue);
    }

    /**
     * This method handles strafing right
     *
     * @param robot - reference to the actual robot
     */
    public static void right(final HardwareNut robot) {

        //Sequence: rD +, lD -, rA -, lA +
        robot.rightDrive.setPower(powerValue);
        robot.leftDrive.setPower(powerValue);
        robot.rightArm.setPower(powerValue);
        robot.leftArm.setPower(powerValue);
    }
}
