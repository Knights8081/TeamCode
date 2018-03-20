package org.firstinspires.ftc.teamcode.teamCode.forLater;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamCode.HardwareNut;
import org.firstinspires.ftc.teamcode.teamCodeGCS.MoveClaw;
import org.firstinspires.ftc.teamcode.teamCodeGCS.StrafeByHand;

/**
 * Created 10/18/2017
 *
 * @author Anna Field
 */
@TeleOp(name="Nut: MechanumWheelsOnePad_TEST", group="Nut")
public class MechanumWheelsOnePad_TEST extends OpMode {

    private final HardwareNut robot = new HardwareNut();        //reference for robot hardware
    private double[] clawPositions;                             //handles updating positions for the claw
    private double[] handPositions;



    /* Game pad controller reference declarations */
    private double left;
    private double right;
    private double left2;
    private double right2;
    private double RT;
    private double LT;
    private double RT2;
    private double LT2;

    final double DEAD_HOME = .2;

    double          deadPosition     = DEAD_HOME;


    public static final double IDOLHAND_MIN_RANGE   =   0.05;
    public static final double IDOLHAND_MAX_RANGE   =   1.3;
    public static final double IDOLHAND_HOME        =   0.08;
    public static final double IDOL_SPEED           =   0.05;

//    DigitalChannel digitalTouch;

    /**
     * Code to run ONCE when the driver hits INIT
     */






    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
//        digitalTouch.setMode(DigitalChannel.Mode.INPUT);




        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /**
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /**
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
//        Move.runMovementTest(robot);  //This is a test function that only is run for debugging issues involving movement
    }

    @Override
    public void loop() {



        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)




        /* SET REFERENCES -----------------------------------------------------------------------*/
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
//        left2 = gamepad1.left_stick_y;
//        right2 = gamepad2.right_stick_y;
        RT = gamepad1.right_trigger;
        LT = gamepad1.left_trigger;
        RT2 = gamepad2.right_trigger;
        LT2 = gamepad2.left_trigger;


        /* SET WHEEL POWER ----------------------------------------------------------------------*/
//        robot.getLeftDrive().setPower(left);
//        robot.getRightDrive().setPower(right);
//        robot.getLeftArm().setPower(left);
//        robot.getRightArm().setPower(right);






        //TODO: MARK FIX THIS
//                if (gamepad1.a)
//                armPosition += ARM_SPEED;
//            else if (gamepad1.y)
//                armPosition -= ARM_SPEED;
//
//
//        armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
//        robot.arm.setPosition(armPosition);






//
//        if (gamepad1.dpad_right){
//            robot.getIdolSlide().setPower(.8);
//        }
//        else if (gamepad1.dpad_left){
//            robot.getIdolSlide().setPower(-.8);
//        }
//        else{
//            robot.getIdolSlide().setPower(0);
//        }

        /* CHECK FOR CLAW UPDATE ----------------------------------------------------------------*/
        clawPositions = robot.getClawPositions();
        handPositions = robot.getIdolHandPosition();

        if (gamepad1.b)
            MoveClaw.closeClaw(clawPositions, HardwareNut.CLAW_SPEED);
        else if (gamepad1.x)
            MoveClaw.openClaw(clawPositions, HardwareNut.CLAW_SPEED);

        if (gamepad1.right_bumper){
            robot.getIdolHand().setPosition(1.0);
        }
        else if (gamepad1.left_bumper){
            robot.getIdolHand().setPosition(0.0);
        }


//        if (gamepad1.dpad_right) {
//            robot.getBallarm().setPower(.75);
//        }
//        else if (gamepad1.dpad_left) {
//            robot.getBallarm().setPower(-.75);
//        }
//        else
//            robot.getBallarm().setPower(0.0);
//
//        if (gamepad1.b)
//            MoveClaw.(handPositions, HardwareNut.CLAW_SPEED);
//        else if (gamepad1.x)
//            MoveClaw.openClaw(handPositions, HardwareNut.CLAW_SPEED);

        if (gamepad1.y) {
            robot.getIdolLift().setPower(-.6);
        }
        else if (gamepad1.a) {
            robot.getIdolLift().setPower(.6);
        }
        else
            robot.getIdolLift().setPower(0);



//        robot.getIdolHand().setPosition(handPositions[0]);

        robot.gettopLeftClaw().setPosition(clawPositions[0]);
        robot.getLeftClaw().setPosition(clawPositions[0]);
        robot.getRightClaw().setPosition(clawPositions[1]);
        robot.gettopRightClaw().setPosition(clawPositions[1]);


        if (gamepad1.right_bumper)
            robot.getIdolHand().setPosition(.9);
        else if (gamepad1.left_bumper)
            robot.getIdolHand().setPosition(.2);

//        handPosition  = Range.clip(handPosition, HardwareNut.IDOLHAND_MIN_RANGE, HardwareNut.IDOLHAND_MAX_RANGE);
//        robot.getIdolHand().setPosition(handPosition);


        /* SET ARM POWER ------------------------------------------------------------------------*/
       robot.getLiftArm().setPower(.1*-left2);


        /* CHECK FOR IDOL SLIDE UPDATE ----------------------------------------------------------*/
        if (gamepad1.dpad_up) {
            robot.getLiftArm().setPower(-.90);
        }
        else if (gamepad1.dpad_down) {
            robot.getLiftArm().setPower(.55);

        }
        else {
            robot.getLiftArm().setPower(0.0);
        }


        /* Strafe Test */
//
//        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
//        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
//        double rightX = gamepad1.right_stick_x;
//        final double v1 = r * Math.cos(robotAngle) + rightX;
//        final double v2 = r * Math.sin(robotAngle) - rightX;
//        final double v3 = r * Math.sin(robotAngle) + rightX;
//        final double v4 = r * Math.cos(robotAngle) - rightX;
//
//        robot.getLeftDrive().setPower(v1);
//        robot.getRightDrive().setPower(v2);
//        robot.getLeftArm().setPower(v3);
//        robot.getRightArm().setPower(v4);






        /* Strafe Test 2 */


        double drive;   // Power for forward and back motion
        double strafe;  // Power for left and right motion
        double rotate;  // Power for rotating the robot
        double frontLeftPower;
        double backLeftPower;
        double frontRightPower;
        double backRightPower;


        drive = -gamepad1.left_stick_y;  // Negative because the gamepad is weird
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        frontLeftPower = drive + strafe + rotate;
        backLeftPower = drive - strafe + rotate;
        frontRightPower = drive - strafe - rotate;
        backRightPower = drive + strafe - rotate;

        robot.getLeftDrive().setPower(-frontLeftPower);
        robot.getRightDrive().setPower(-frontRightPower);
        robot.getLeftArm().setPower(-backLeftPower);
        robot.getRightArm().setPower(-backRightPower);


        /* CHECK FOR STRAFING -------------------------------------------------------------------*/
//             if (RT >= 0.1) {
//                StrafeByHand.right(robot, RT);
//            } else if (LT >= 0.1) {
//                StrafeByHand.left(robot, LT);
//           }


//    if (gamepad1.right_bumper && !digitalTouch.getState() == false) {
//        robot.getIdolLift().setPower(0.0);
//    }

}


}
