package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by afield on 10/18/2017.
 */
@TeleOp(name="Nut: MechanumWheels", group="Nut")

public class MechanumWheels extends OpMode {
    /* Declare OpMode members. */
    HardwareNut robot = new HardwareNut(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double          armPosition     = robot.ARM_HOME;                   // Servo safe position
    double          clawPosition    = robot.CLAW_HOME;
    double          handPosition    = robot.IDOLHAND_HOME;
    final double    ARM_SPEED       = 0.01 ;                            // sets rate to move servo

    double clawOffset = 0.5;                  // Servo mid position
    final double CLAW_SPEED = 0.02;// sets rate to move servo
    final double Idol_SPEED = 0.02;
    double RT = 0.0;
    double LT = 0.0;
    double lift_up = 0.0;


    /*
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

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {


    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    }

    @Override
    public void loop() {
        double left;
        double right;
        double left2;
        double right2;
        double RT;
        double LT;
        double RT2;
        double LT2;






        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        left2 = gamepad2.left_stick_y;
        right2 = gamepad2.right_stick_y;
        RT = gamepad1.right_trigger;
        LT = gamepad1.left_trigger;
        RT2 = gamepad2.right_trigger;
        LT2 = gamepad2.left_trigger;



        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);
        robot.leftArm.setPower(left);
        robot.rightArm.setPower(right);



        // Use gamepad left & right Bumpers to open and close the claw
        //if (gamepad2.right_bumper){
            //robot.leftClaw = clawOffset += CLAW_SPEED;}
            //robot.leftClaw = +CLAW_SPEED;}

       // else if (gamepad2.left_bumper){
            //robot.rightClaw = clawOffset -= CLAW_SPEED;}
            //robot.rightClaw = -CLAW_SPEED;}

        robot.glyph.setPower(right2);

        while (gamepad2.dpad_right){
            robot.idollift.setPower(.5);}
        if (gamepad2.dpad_up){
            robot.idollift.setPower(0);}
        while (gamepad2.dpad_left){
            robot.idollift.setPower(-.5);}







        if (gamepad1.x)
            clawPosition += CLAW_SPEED;
        else if (gamepad1.b)
            clawPosition -= CLAW_SPEED;

        if (gamepad2.x)
            handPosition += Idol_SPEED;
        else if (gamepad2.b)
            handPosition -= Idol_SPEED;

        handPosition  = Range.clip(handPosition, robot.IDOLHAND_MIN_RANGE, robot.IDOLHAND_MAX_RANGE);
        robot.idolhand.setPosition(handPosition);
        armPosition  = Range.clip(armPosition, robot.ARM_MIN_RANGE, robot.ARM_MAX_RANGE);
        robot.arm.setPosition(armPosition);
        clawPosition = Range.clip(clawPosition, robot.CLAW_MIN_RANGE, robot.CLAW_MAX_RANGE);
        robot.claw.setPosition(clawPosition);



        robot.liftArm.setPower(left2);



        if (gamepad2.right_trigger > 0.1) {
            robot.idolslide.setPower(.5*RT2);
        }
        else if (gamepad2.left_trigger > 0.1){
            robot.idolslide.setPower(-.5*LT2);
        }

        if (gamepad1.right_trigger > 0.1) {
            robot.rightDrive.setPower(.75*RT);
            robot.leftDrive.setPower(.75*-RT);
            robot.rightArm.setPower(.75*-RT);
            robot.leftArm.setPower(.75*RT);

        }

        else if (gamepad1.left_trigger > 0.1){
            robot.rightDrive.setPower(.75*-LT);
            robot.leftDrive.setPower(.75*LT);
            robot.rightArm.setPower(.75*LT);
            robot.leftArm.setPower(.75*-LT);}


    }

    }

