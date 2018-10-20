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

package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link MySensorMRRangeSensor} illustrates how to use the Modern Robotics
 * Range Sensor.
 * <p>
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 * <p>
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
@Autonomous(name = "MySensor: MR range sensor", group = "Sensor")
//@Disabled   // comment out or remove this line to enable this opmode
public class MySensorMRRangeSensor extends LinearOpMode {

    MyHardwarePushbot robot = new MyHardwarePushbot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, this , "autonomous");
        // get a reference to our compass
        telemetry.addData("Say", "init");
        telemetry.update();
        // wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Say", "before enoder");
            robot.encoderDrive(0.5, 5, 5, 0.2);
            telemetry.addData("Say", "after encoder");

            if (robot.rangeSensor.cmOptical() > 6) {
                robot.encoderDrive(0.5, 6, 2.5, 0.2);
                telemetry.addData("Say", "Too far");
                telemetry.update();


            }
            else if (robot.rangeSensor.cmOptical() < 5) {
                robot.encoderDrive(0.5, 2.5, 6, 0.2);
                telemetry.addData("Say", "Too close");
                telemetry.update();

            }
            else{
                robot.encoderDrive(0.5, 5, 5, 0.2);
                telemetry.addData("Say", "Forward");
                telemetry.update();

            }
            telemetry.addData("raw ultrasonic",robot.rangeSensor.rawUltrasonic());
            telemetry.addData("raw optical",robot.rangeSensor.rawOptical());
            telemetry.addData("cm optical","%.2f cm",robot.rangeSensor.cmOptical());
            telemetry.addData("cm","%.2f cm",robot.rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}
