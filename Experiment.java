/* Copyright (c) 2015 Craig MacFarlane

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Craig MacFarlane nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by 35854 on 8/11/2016.
 */

@TeleOp(name = "Experiment_TeleOP", group = "TeleOp")

public class Experiment extends OpMode
{

    final static double ARM_MIN_RANGE  = 0.01;
    final static double ARM_MAX_RANGE  = 0.95;
    final static double ARM2_MIN_RANGE = 0.01;
    final static double ARM2_MAX_RANGE = 0.95;
    double armPosition;
    double arm2Position;

    double armDelta = 0.01;
    double armTheta = 0.01;


    DcMotor motorRight;
    DcMotor motorRight2;
    DcMotor motorLeft;
    DcMotor motorLeft2;
    Servo Arm;
    Servo Arm2;


    @Override
    public void init() {


        motorLeft = hardwareMap.dcMotor.get("left_drive");
        motorLeft2 = hardwareMap.dcMotor.get("left_drive2");
        motorRight = hardwareMap.dcMotor.get("right_drive");
        motorRight2 = hardwareMap.dcMotor.get("right_drive2");
        Arm = hardwareMap.servo.get("servo_arm");
        Arm2 = hardwareMap.servo.get("servo_arm2");
        motorRight.setDirection(DcMotor.Direction.REVERSE);
        motorRight2.setDirection(DcMotor.Direction.REVERSE);



    }

    @Override
    public void loop() {

        motorLeft.setPower(-gamepad1.left_stick_y);
        motorLeft2.setPower(-gamepad1.left_stick_y);
        motorRight.setPower(-gamepad1.right_stick_y);
        motorRight2.setPower(-gamepad1.right_stick_y);

        if (gamepad1.a) {
            Arm.setPosition(0.7);
        }

        else if (gamepad1.y) {
            Arm.setPosition(0.2);
        }

        else {
            Arm.setPosition(0.5);
        }


        if (gamepad1.b) {
            arm2Position = arm2Position + armTheta;
        }

        if (gamepad1.x) {
            arm2Position = arm2Position - armTheta;

        }




        armPosition = Range.clip(armPosition, ARM_MIN_RANGE, ARM_MAX_RANGE);
        arm2Position = Range.clip(arm2Position, ARM2_MIN_RANGE, ARM2_MAX_RANGE);

        Arm2.setPosition(arm2Position);

        telemetry.addData("3-arm position", armPosition);

    }
}
