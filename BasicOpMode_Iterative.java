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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name="Skystone Auto", group="Iterative Opmode")
//@Disabled
public class BasicOpMode_Auto extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FL = null;
    private DcMotor BL = null;
    private DcMotor FR = null;
    private DcMotor BR = null;
    private DcMotor AV = null;
    private DcMotor AH = null;
    private Servo servo = null;

    private double positionAV = 0;
    private double UpLimAV = 5;
    private double DownLimAV = -1;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        servo = hardwareMap.servo.get("CS");
        servo.setPosition(0.6);
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FL  = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL  = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        AV = hardwareMap.get(DcMotor.class, "AV");
        AH = hardwareMap.get(DcMotor.class, "AH");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        BL.setDirection(DcMotor.Direction.FORWARD);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.FORWARD);
        FR.setDirection(DcMotor.Direction.FORWARD);
        AV.setDirection(DcMotor.Direction.FORWARD);
        AH.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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

        runtime.reset();

        try {
            forward(1000, 0);
//            leftwards(1000, 0);
//            backwards(1000, 1);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    public void setPowerAmount(double x, double y, double turn, double speed)
    {
        FL.setPower(Range.clip( speed*((x+y)/2) + turn/4  , -1.0, 1.0));
        FR.setPower(Range.clip( speed*((x-y)/2) + turn/4  , -1.0, 1.0));
        BR.setPower(Range.clip( speed*((-x-y)/2) + turn/4  , -1.0, 1.0));
        BL.setPower(Range.clip( speed*((-x+y)/2) + turn/4  , -1.0, 1.0));
    }

    public void backwards(int milliseconds, double turn) throws InterruptedException {
        setPowerAmount(0, 1, turn, 1);
        Thread.sleep(milliseconds);
        hault();
    }
    public void leftwards(int milliseconds, double turn) throws InterruptedException {
        setPowerAmount(1, 0, turn, 1);
        Thread.sleep(milliseconds);
        hault();
    }
    public void rightwards(int milliseconds, double turn) throws InterruptedException {
        setPowerAmount(-1, -1, turn, 1);
        Thread.sleep(milliseconds);
        hault();
    }
    public void forward(int milliseconds, double turn) throws InterruptedException {
        setPowerAmount(0, -1, turn, 1);
        Thread.sleep(milliseconds);
        hault();
    }

    public void turn(int milliseconds, double turn) throws InterruptedException {
        setPowerAmount(0, 0, turn, 1);
        Thread.sleep(milliseconds);
        hault();
    }

    public void hault()
    {
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        AV.setPower(0);
        AH.setPower(0);
    }



    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
//        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        FL.setPower(0);
        BL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
        AV.setPower(0);
        AH.setPower(0);
    }
//
//    public void limitCountAV(double input) {
//        if(Math.abs(input) > 0.5) {
//            positionAV += input;
//        }
//
//        if(positionAV < UpLimAV || positionAV > DownLimAV) {
//            AV.setPower(input);
//        } else {
//            AV.setPower(0);
//        }
//    }

}
