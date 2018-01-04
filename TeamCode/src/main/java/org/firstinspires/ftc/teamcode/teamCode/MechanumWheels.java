package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamCode.HardwareNut;
import org.firstinspires.ftc.teamcode.teamCodeGCS.StrafeByHand;
import org.firstinspires.ftc.teamcode.testClasses.movement.Move;

/**
 * Created 10/18/2017
 *
 * @author Anna Field
 */
@TeleOp(name="Nut: MechanumWheels", group="Nut")
public class MechanumWheels extends OpMode {
    /* Declare OpMode members. */
    private final HardwareNut robot = new HardwareNut(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    private double          leftClawPosition    = HardwareNut.CLAW_HOME;
    private double          rightClawPosition    = HardwareNut.CLAW_HOME;
    private double          handPosition    = HardwareNut.IDOLHAND_HOME;
//    final double    CLAW_SPEED       = 0.01 ;                            // sets rate to move servo

//    double clawOffset = 0.5;                  // Servo mid position
    private final double CLAW_SPEED = 0.02;             // sets rate to move servo
    private final double Idol_SPEED = 0.02;
    private double left;
    private double right;
    private double left2;
    private double right2;
    private double RT;
    private double LT;
    private double RT2;
    private double LT2;


    /**
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

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
//        Move.runMovementTest(robot);
    }

    @Override
    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        left2 = gamepad2.left_stick_y;
        right2 = gamepad2.right_stick_y;
        RT = gamepad1.right_trigger;
        LT = gamepad1.left_trigger;
        RT2 = gamepad2.right_trigger;
        LT2 = gamepad2.left_trigger;



        robot.getLeftDrive().setPower(left);
        robot.getRightDrive().setPower(right);
        robot.getLeftArm().setPower(left);
        robot.getRightArm().setPower(right);



        // Use gamepad left & right Bumpers to open and close the claw
        //if (gamepad2.right_bumper){
            //robot.leftClaw = clawOffset += CLAW_SPEED;}
            //robot.leftClaw = +CLAW_SPEED;}

       // else if (gamepad2.left_bumper){
            //robot.rightClaw = clawOffset -= CLAW_SPEED;}
            //robot.rightClaw = -CLAW_SPEED;}

//        robot.getGlyph().setPower(right2);



//        while (gamepad2.dpad_right){
//            robot.getIdolLift().setPower(.5);}
//        if (gamepad2.dpad_up){
//            robot.getIdolLift().setPower(0);}
//        while (gamepad2.dpad_left){
//            robot.getIdolLift().setPower(-.5);}



        if (gamepad2.x) {
            leftClawPosition -= CLAW_SPEED;
            rightClawPosition -= CLAW_SPEED;

        }

        else if (gamepad2.b) {
            leftClawPosition += CLAW_SPEED;
            rightClawPosition += CLAW_SPEED;

        }

        leftClawPosition  = Range.clip(leftClawPosition, HardwareNut.CLAW_MIN_RANGE, HardwareNut.CLAW_MAX_RANGE);
        rightClawPosition = Range.clip(rightClawPosition, HardwareNut.CLAW_MIN_RANGE, HardwareNut.CLAW_MAX_RANGE);

        robot.getLeftClaw().setPosition(leftClawPosition);
        robot.getRightClaw().setPosition(rightClawPosition);

//        if (gamepad2.a)
//            handPosition += Idol_SPEED;
//        else if (gamepad2.y)
//            handPosition -= Idol_SPEED;





        handPosition  = Range.clip(handPosition, robot.IDOLHAND_MIN_RANGE, robot.IDOLHAND_MAX_RANGE);
        robot.getIdolHand().setPosition(handPosition);




        robot.getLiftArm().setPower(left2);



        if (RT2 > 0.1)
            robot.getIdolSlide().setPower(.5*RT2);
        else if (LT2 > 0.1)
            robot.getIdolSlide().setPower(-.5*LT2);

        if (RT > 0.1)
            StrafeByHand.right(robot, RT);
        else if (LT > 0.1)
            StrafeByHand.left(robot, LT);


    }
}
