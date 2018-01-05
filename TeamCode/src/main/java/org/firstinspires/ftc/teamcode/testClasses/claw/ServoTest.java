package org.firstinspires.ftc.teamcode.testClasses.claw;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teamCode.HardwareNut;
import org.firstinspires.ftc.teamcode.teamCodeGCS.MoveClaw;

/**
 * @author Luke Frazer
 */
@TeleOp(name="Nut: Servo test 1", group="Nut")
public class ServoTest extends OpMode {

    private final HardwareNut robot = new HardwareNut();    //reference for robot hardware

    /**
     * Keeps track of the positions for the two servos that control the claw
     *
     *  positions[0] -> left claw position
     *  positions[1] -> right claw position
     */
    private final double[] positions = {HardwareNut.CLAW_HOME, HardwareNut.CLAW_HOME};

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
    }

    @Override
    public void loop() {

        if (gamepad2.x)
            MoveClaw.closeClaw(positions, HardwareNut.CLAW_SPEED);
        else if (gamepad2.b)
            MoveClaw.openClaw(positions, HardwareNut.CLAW_SPEED);

        setPositions();
    }

    private void setPositions() {
        robot.getLeftClaw().setPosition(positions[0]);
        robot.getRightClaw().setPosition(positions[1]);
    }
}
