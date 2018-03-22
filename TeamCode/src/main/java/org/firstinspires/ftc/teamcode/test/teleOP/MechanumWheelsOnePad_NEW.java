package org.firstinspires.ftc.teamcode.test.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pd.HardwareNut;
import org.firstinspires.ftc.teamcode.pd.teamCodeGCS.DrivingMove;
import org.firstinspires.ftc.teamcode.pd.teamCodeGCS.MoveClaw;

/**
 * Created 10/18/2017
 *
 * @author Anna Field
 */
@TeleOp(name="Nut: MechanumWheelsOnePad_NEW", group="Nut")
public class MechanumWheelsOnePad_NEW extends OpMode {

    private final HardwareNut robot = new HardwareNut();        //reference for robot hardware
    private double[] clawPositions;                             //handles updating positions for the claw
    private double[] handPositions;

    /* Game pad controller reference declarations */
    private double left;
    private double right;
    private double left2;
    private double RT;
    private double LT;

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
        RT = gamepad1.right_trigger;
        LT = gamepad1.left_trigger;




        /* CHECK FOR CLAW UPDATE ----------------------------------------------------------------*/
        clawPositions = robot.getClawPositions();
        handPositions = robot.getIdolHandPosition();

        if (gamepad1.b)
            MoveClaw.closeClaw(clawPositions, HardwareNut.CLAW_SPEED);
        else if (gamepad1.x)
            MoveClaw.openClaw(clawPositions, HardwareNut.CLAW_SPEED);



        if (gamepad1.right_bumper)
            robot.getIdolHand().setPosition(1.0);
        else if (gamepad1.left_bumper)
            robot.getIdolHand().setPosition(0.0);


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

        if (gamepad1.y)
            robot.getIdolLift().setPower(-.6);
        else if (gamepad1.a)
            robot.getIdolLift().setPower(.6);
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
        if (gamepad1.dpad_up)
            robot.getLiftArm().setPower(-.90);
        else if (gamepad1.dpad_down)
            robot.getLiftArm().setPower(.55);
        else
            robot.getLiftArm().setPower(0.0);


        /* DRIVE ROBOT --------------------------------------------------------------------------*/
        double drive = -gamepad1.left_stick_y;   // Power for forward and back motion; Negative because the gamepad is weird
        double strafe = gamepad1.left_stick_x;  // Power for left and right motion
        double rotate = gamepad1.right_stick_x;  // Power for rotating the robot

        DrivingMove.drive(robot, drive, strafe, rotate);
    }
}
