package org.firstinspires.ftc.teamcode.teamCodeGCS;

import org.firstinspires.ftc.teamcode.teamCode.HardwareNut;

public final class StrafeByHand {

    private static final float value = 0.75f;

    public static void left(final HardwareNut robot, final double trigger) {

        //Sequence: rD -, lD +, rA +, lA -
        robot.rightDrive.setPower(value * -trigger);
        robot.leftDrive.setPower(value * trigger);
        robot.rightArm.setPower(value * trigger);
        robot.leftArm.setPower(value * -trigger);

    }

    public static void right(final HardwareNut robot, final double trigger) {

        //Sequence: rD +, lD -, rA -, lA +
        robot.rightDrive.setPower(value * trigger);
        robot.leftDrive.setPower(value * -trigger);
        robot.rightArm.setPower(value * -trigger);
        robot.leftArm.setPower(value * trigger);
    }
}