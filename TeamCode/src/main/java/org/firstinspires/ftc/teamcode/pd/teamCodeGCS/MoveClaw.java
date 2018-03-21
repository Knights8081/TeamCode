package org.firstinspires.ftc.teamcode.pd.teamCodeGCS;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.pd.HardwareNut;

/**
 * This class contains functions to update the positions for the hands on the claw
 *
 * @author Corbin Young
 */
public final class MoveClaw {

    /**
     * This function moves the arms of the claw outward so that the claw "opens" and lets go
     *  of whatever object it may or may not have been holding onto
     *
     *  positions[0] -> left claw position
     *  positions[1] -> right claw position
     *
     * @param positions - array holding the positions for the hands for the claw
     * @param speed - speed at which the claw will change positions
     */
    public static void openClaw(final double[] positions, final double speed) {
        /* Update positions */
        positions[0] += speed;
        positions[1] -= speed;

        /* Clip the positions to make sure they are within the valid range for the servos */
        positions[0]  = Range.clip(positions[0], HardwareNut.CLAW_MIN_RANGE, HardwareNut.CLAW_MAX_RANGE);
        positions[1] = Range.clip(positions[1], HardwareNut.CLAW_MIN_RANGE, HardwareNut.CLAW_MAX_RANGE);
    }

    /**
     * This function moves the arms of the claw inward so that the claw "closes" and grabs
     *  whatever object may or may not be in front of it
     *
     *  positions[0] -> left claw position
     *  positions[1] -> right claw position
     *
     * @param positions - array holding the positions for the hands for the claw
     * @param speed - speed at which the claw will change positions
     */
    public static void closeClaw(final double[] positions, final double speed) {
        /* Update positions */
        positions[0] -= speed;
        positions[1] += speed;

        /* Clip the positions to make sure they are within the valid range for the servos */
        positions[0]  = Range.clip(positions[0], HardwareNut.CLAW_MIN_RANGE, HardwareNut.CLAW_MAX_RANGE);
        positions[1] = Range.clip(positions[1], HardwareNut.CLAW_MIN_RANGE, HardwareNut.CLAW_MAX_RANGE);
    }
}
