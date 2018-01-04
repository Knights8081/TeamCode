package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamCodeGCS.MoveClaw;

/**
 * @author Luke Frazer
 */
@TeleOp(name="Nut: Servo test", group="Nut")
public class ServoTest extends OpMode {
    private final HardwareNut robot = new HardwareNut();

    private double          leftClawPosition    = HardwareNut.CLAW_HOME;
    private double          rightClawPosition    = HardwareNut.CLAW_HOME;
    private double          handPosition    = HardwareNut.IDOLHAND_HOME;

    private final double CLAW_SPEED = 0.02;

    private final double[] positions = {leftClawPosition, rightClawPosition};

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
            MoveClaw.closeClaw(positions, CLAW_SPEED);
        else if (gamepad2.b)
            MoveClaw.openClaw(positions, CLAW_SPEED);

        setPositions();
    }

    private void setPositions() {
        robot.getLeftClaw().setPosition(positions[0]);
        robot.getRightClaw().setPosition(positions[1]);
    }
}
