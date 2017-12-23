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
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class HardwareNut
{
    /* Public OpMode members. */
    public DcMotor  leftDrive   = null;
    public DcMotor  rightDrive  = null;
    public DcMotor  leftArm     = null;
    public DcMotor  rightArm    = null;
    public Servo    arm         = null;
    public Servo    claw        = null;
    public Servo    idolhand    = null;
    public DcMotor  liftArm     = null;
    public DcMotor  idolslide   = null;
    public DcMotor  idollift    = null;
    public DcMotor  glyph       = null;

    public DcMotor FR = null;
    public DcMotor FL = null;
    public DcMotor BR = null;
    public DcMotor BL = null;

    public static final double MID_SERVO       =  0.0 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    public static final double IDOLHAND_MIN_RANGE  = 0.05 ;
    public static final double IDOLHAND_MAX_RANGE = 1.3 ;
    public final static double CLAW_MIN_RANGE  = 0.05;
    public final static double CLAW_MAX_RANGE  = 1.3;
    public final static double ARM_MIN_RANGE  = 0.05;
    public final static double ARM_MAX_RANGE  = 1.0;

    public final static double ARM_HOME = 0.08;
    public final static double CLAW_HOME = 0.08;
    public final static double IDOLHAND_HOME = 0.08;




    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareNut(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotor.class, "left_drive");
        rightDrive = hwMap.get(DcMotor.class, "right_drive");
        leftArm    = hwMap.get(DcMotor.class, "left_arm");
        rightArm   = hwMap.get(DcMotor.class, "right_arm");
        liftArm = hwMap.get(DcMotor.class, "lift_arm");
        idolslide = hwMap.get(DcMotor.class, "idol_slide");
        glyph = hwMap.get(DcMotor.class, "glyph_hand");
        idollift = hwMap.get(DcMotor.class, "idol_lift");
        leftDrive.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        leftArm.setDirection(DcMotor.Direction.FORWARD);
        rightArm.setDirection(DcMotor.Direction.REVERSE);
        idolslide.setDirection(DcMotor.Direction.FORWARD);
        glyph.setDirection(DcMotor.Direction.FORWARD);



        FR = hwMap.get(DcMotor.class, "right_drive");
        FL = hwMap.get(DcMotor.class, "left_drive");
        BR = hwMap.get(DcMotor.class, "right_drive");
        BL = hwMap.get(DcMotor.class, "right_drive");




   //** //leftDrive = FrontLeft
        //rightDrive = FrontRight
        //leftArm = Backleft
        //rightArm = Backright



        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        leftArm.setPower(0);
        rightArm.setPower(0);
        liftArm.setPower(0);
        FR.setPower(0);
        FL.setPower(0);
        BR.setPower(0);
        BL.setPower(0);
        idolslide.setPower(0);
        idollift.setPower(0);
        glyph.setPower(0);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idolslide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idollift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glyph.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




        // Define and initialize ALL installed servos.
        claw  = hwMap.get(Servo.class, "left_hand");
        arm = hwMap.get(Servo.class, "right_hand");
        idolhand = hwMap.get(Servo.class, "idol_hand");
        claw.setPosition(MID_SERVO);
        arm.setPosition(MID_SERVO);







    }


}
