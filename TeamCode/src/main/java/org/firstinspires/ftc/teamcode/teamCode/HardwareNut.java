/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teamCode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareNut {

    /* Define Motor references ------------------------------------------------------------------*/
    private DcMotor  leftDrive   = null;        // Front left wheel
    private DcMotor  rightDrive  = null;        // Front right wheel
    private DcMotor  leftArm     = null;        // Back left wheel
    private DcMotor  rightArm    = null;        // Back right wheel
    private DcMotor  liftArm     = null;
    private DcMotor  idolSlide   = null;
    private DcMotor  idolLift    = null;
    private DcMotor  glyph       = null;

    /* Define Servo references ------------------------------------------------------------------*/
    private Servo    leftClaw    = null;
    private Servo    rightClaw   = null;
    private Servo    idolHand    = null;

    /* Define global constants ------------------------------------------------------------------*/

    /* Arm Constants */
    public static final double ARM_UP_POWER         =   0.45;
    public static final double ARM_DOWN_POWER       =  -0.45;

    /* Servo Constants */
    public static final double CLAW_MIN_RANGE       =   0.05;
    public static final double CLAW_MAX_RANGE       =   1.3;
    public static final double CLAW_HOME            =   0.08;
    public static final double CLAW_SPEED           =   0.05;
    public static final double MID_SERVO            =   0.0;

    /* Idol Hand Constants */
    public static final double IDOLHAND_MIN_RANGE   =   0.05;
    public static final double IDOLHAND_MAX_RANGE   =   1.3;
    public static final double IDOLHAND_HOME        =   0.08;
    public static final double IDOL_SPEED           =   0.05;


    /**
     * Creates an instance of the Hardware Nut class
     */
    public HardwareNut(){
    }

    /* Motor getters ----------------------------------------------------------------------------*/
    public DcMotor getLeftDrive() {
        return leftDrive;
    }

    public DcMotor getRightDrive() {
        return rightDrive;
    }

    public DcMotor getLeftArm() {
        return leftArm;
    }

    public DcMotor getRightArm() {
        return rightArm;
    }

    public DcMotor getLiftArm() {
        return liftArm;
    }

    public DcMotor getIdolSlide() {
        return idolSlide;
    }

    public DcMotor getIdolLift() {
        return idolLift;
    }

    public DcMotor getGlyph() {
        return glyph;
    }

    /* Servo getters ----------------------------------------------------------------------------*/
    public Servo getLeftClaw() {
        return leftClaw;
    }

    public Servo getRightClaw() {
        return rightClaw;
    }

    public Servo getIdolHand() {
        return idolHand;
    }

    public double[] getClawPositions() {
        return new double[]{leftClaw.getPosition(), rightClaw.getPosition()};
    }

    public double getIdolHandPosition() {
        return idolHand.getPosition();
    }

    /* Functional methods -----------------------------------------------------------------------*/
    /**
     * Initialize all standard hardware interfaces
     *
     * @param hwMap - reference to the hardware map on the user interface program
     */
    public void init(final HardwareMap hwMap) {

        /* INITIALIZE MOTORS --------------------------------------------------------------------*/

        /* Wheel Motors */
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftArm    = hwMap.get(DcMotor.class, "left_arm");
        rightArm   = hwMap.get(DcMotor.class, "right_arm");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);  // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE); // Set to FORWARD if using AndyMark motors
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);


        /* Arm Motors */
        liftArm = hwMap.get(DcMotor.class, "lift_arm");
        idolSlide = hwMap.get(DcMotor.class, "idol_slide");
        glyph = hwMap.get(DcMotor.class, "glyph_hand");
        idolLift = hwMap.get(DcMotor.class, "idol_lift");

        idolSlide.setDirection(DcMotor.Direction.FORWARD);
        glyph.setDirection(DcMotor.Direction.FORWARD);

//        TODO - Why are these here? Are we using them anywhere but here?
//        DcMotor FR = hwMap.get(DcMotor.class, "right_drive");
//        DcMotor FL = hwMap.get(DcMotor.class, "left_drive");
//        DcMotor BR = hwMap.get(DcMotor.class, "right_drive");
//        DcMotor BL = hwMap.get(DcMotor.class, "right_drive");



        // Set all motors to zero power
        /* SET INITIAL POWER --------------------------------------------------------------------*/
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftArm.setPower(0);
        rightArm.setPower(0);
        liftArm.setPower(0);
//        FR.setPower(0);
//        FL.setPower(0);
//        BR.setPower(0);
//        BL.setPower(0);
        idolSlide.setPower(0);
        idolLift.setPower(0);
        glyph.setPower(0);


        /* SET MOTOR MODE -----------------------------------------------------------------------*/
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idolSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idolLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glyph.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        /* INITIALIZE SERVOS --------------------------------------------------------------------*/
        rightClaw  = hwMap.get(Servo.class, "right_claw");
        leftClaw = hwMap.get(Servo.class, "left_claw");
        idolHand = hwMap.get(Servo.class, "idol_hand");

        rightClaw.setPosition(CLAW_MIN_RANGE);
        leftClaw.setPosition(CLAW_MAX_RANGE);
    }
}

