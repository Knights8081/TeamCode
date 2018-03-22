package org.firstinspires.ftc.teamcode.pd.teamCodeGCS;

import org.firstinspires.ftc.teamcode.pd.HardwareNut;

/**
 * Created by afield on 3/20/2018.
 */
public final class DrivingMove {

    public static void drive(final HardwareNut robot, final double drive, final double strafe, final double rotate) {
        double frontLeftPower = drive + strafe + rotate;
        double backLeftPower = drive - strafe + rotate;
        double frontRightPower = drive - strafe - rotate;
        double backRightPower = drive + strafe - rotate;

        robot.getLeftDrive().setPower(-frontLeftPower);
        robot.getRightDrive().setPower(-frontRightPower);
        robot.getLeftArm().setPower(-backLeftPower);
        robot.getRightArm().setPower(-backRightPower);
    }
}
