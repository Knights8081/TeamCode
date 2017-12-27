package org.firstinspires.ftc.teamcode.teamCode.forLater;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamCode.HardwareNut;

/**
 * Created by afield on 10/18/2017.
 */
@TeleOp(name="Nut: MechanumWheels2", group="Nut")

public class MechanumWheels2 extends OpMode {
    /* Declare OpMode members. */
    HardwareNut robot = new HardwareNut(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double clawOffset = 0.0;                  // Servo mid position
    final double CLAW_SPEED = 0.02;             // sets rate to move servo
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
        double RT;
        double LT;




        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = gamepad1.left_stick_y;
        right = gamepad1.right_stick_y;
        RT = gamepad1.right_trigger;
        LT = gamepad1.left_trigger;



        robot.leftDrive.setPower(left);
        robot.rightDrive.setPower(right);
        robot.leftArm.setPower(left);
        robot.rightArm.setPower(right);



        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;

        if (gamepad2.dpad_up){
            robot.liftArm.setPower(.75);}
        else if (gamepad2.dpad_down){
            robot.liftArm.setPower(-.75);}
        robot.liftArm.setPower(0.0);



        if (gamepad1.right_trigger > 0.1) {
            new StrafeR();
        }

        else if (gamepad1.left_trigger > 0.1) {
            new StrafeL();
        }


    }

}

