package org.firstinspires.ftc.teamcode.teamCodeGCS;

import org.firstinspires.ftc.teamcode.teamCode.HardwareNut;

/**
 * This class contains two small macro functions to handle strafing based on input from the
 *  controller.
 *
 * @author Luke Frazer
 */
public final class StrafeByHand {

    //We want to use 75% of the triggers value when setting power
    private static final float percentage = 0.75f;

    /**
     * This method handles strafing left
     *
     * @param robot - reference to the actual robot
     * @param trigger - value given by the controller
     */
    public static void left(final HardwareNut robot, final double trigger) {

        //Sequence: rD -, lD +, rA +, lA -
        robot.rightDrive.setPower(percentage * -trigger);
        robot.leftDrive.setPower(percentage * trigger);
        robot.rightArm.setPower(percentage * trigger);
        robot.leftArm.setPower(percentage * -trigger);
    }

    /**
     * This method handles strafing right
     *
     * @param robot - reference to the actual robot
     * @param trigger - value given by the controller
     */
    public static void right(final HardwareNut robot, final double trigger) {

        //Sequence: rD +, lD -, rA -, lA +
        robot.rightDrive.setPower(percentage * trigger);
        robot.leftDrive.setPower(percentage * -trigger);
        robot.rightArm.setPower(percentage * -trigger);
        robot.leftArm.setPower(percentage * trigger);
    }
}