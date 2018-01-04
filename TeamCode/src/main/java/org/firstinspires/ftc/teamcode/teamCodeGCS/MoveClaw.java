package org.firstinspires.ftc.teamcode.teamCodeGCS;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.teamCode.HardwareNut;

/**
 * @author Corbin Young
 */
public final class MoveClaw {

    /**
     *
     * @param positions
     * @param speed
     */
    public static void openClaw(final double[] positions, final double speed) {
        positions[0] -= speed;
        positions[1] -= speed;

        positions[0]  = Range.clip(positions[0], HardwareNut.CLAW_MIN_RANGE, HardwareNut.CLAW_MAX_RANGE);
        positions[1] = Range.clip(positions[1], HardwareNut.CLAW_MIN_RANGE, HardwareNut.CLAW_MAX_RANGE);
    }

    /**
     *
     * @param positions
     * @param speed
     */
    public static void closeClaw(final double[] positions, final double speed) {
        positions[0] += speed;
        positions[1] += speed;

        positions[0]  = Range.clip(positions[0], HardwareNut.CLAW_MIN_RANGE, HardwareNut.CLAW_MAX_RANGE);
        positions[1] = Range.clip(positions[1], HardwareNut.CLAW_MIN_RANGE, HardwareNut.CLAW_MAX_RANGE);
    }
}
