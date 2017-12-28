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
    private HardwareNut robot = new HardwareNut(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    double          clawPosition    = robot.CLAW_HOME;
    double          handPosition    = robot.IDOLHAND_HOME;
//    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo

//    double clawOffset = 0.5;                  // Servo mid position
    final double CLAW_SPEED = 0.02;// sets rate to move servo
    final double Idol_SPEED = 0.02;
//    double RT = 0.0;
//    double LT = 0.0;
//    double lift_up = 0.0;


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
        double left = gamepad1.left_stick_y;
        double right = gamepad1.right_stick_y;
        double left2 = gamepad2.left_stick_y;
        double right2 = gamepad2.right_stick_y;
        double RT = gamepad1.right_trigger;
        double LT = gamepad1.left_trigger;
        double RT2 = gamepad2.right_trigger;
        double LT2 = gamepad2.left_trigger;



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

        robot.getGlyph().setPower(right2);

        while (gamepad2.dpad_right){
            robot.getIdolLift().setPower(.5);}
        if (gamepad2.dpad_up){
            robot.getIdolLift().setPower(0);}
        while (gamepad2.dpad_left){
            robot.getIdolLift().setPower(-.5);}



        if (gamepad1.x)
            clawPosition += CLAW_SPEED;
        else if (gamepad1.b)
            clawPosition -= CLAW_SPEED;

        if (gamepad2.x)
            handPosition += Idol_SPEED;
        else if (gamepad2.b)
            handPosition -= Idol_SPEED;

        handPosition  = Range.clip(handPosition, robot.IDOLHAND_MIN_RANGE, robot.IDOLHAND_MAX_RANGE);
        robot.getIdolHand().setPosition(handPosition);
        armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
        robot.getArm().setPosition(armPosition);
        clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
        robot.getClaw().setPosition(clawPosition);



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
