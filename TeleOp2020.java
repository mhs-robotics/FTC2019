/* Copyright (c) 2019 FIRST. All rights reserved.
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

package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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

@TeleOp(name="This one idiot", group="Iterative Opmode")
public class TeleOp2019 extends OpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor leftRear = null;
    private DcMotor leftFront = null;
    private DcMotor rightRear = null;
    private DcMotor rightFront = null;
    private DcMotor spinLeft = null;
    private DcMotor spinRight = null;
    private DcMotor spool = null;
    private DcMotor horizontalSlide = null;
    private Servo grabServo = null;
    private Servo rotateServo = null;
    private Servo elementServo = null;
    private Servo leftPlat = null;
    private Servo rightPlat = null;
    private double maxClip, minClip;
    private double rotatePower = 0;
    private int spoolPosition = 0;
    private double grabPosition = 0;
    private int elementState = 0;
    private int platState = 0;
    private int slowMode = 0;
    private double feedPower = 0;
    private double precisionRatio = 0;
    private int manualOverride = 0;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");
        
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRear  = hardwareMap.get(DcMotor.class, "left_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        
        spinLeft = hardwareMap.get(DcMotor.class, "spin_left");
        spinRight = hardwareMap.get(DcMotor.class, "spin_right");

        spool = hardwareMap.get(DcMotor.class, "spool_motor");
        
        horizontalSlide = hardwareMap.get(DcMotor.class, "horizontal_slide");

        grabServo = hardwareMap.get(Servo.class, "grab_servo");
        rotateServo = hardwareMap.get(Servo.class, "rotate_servo");

        leftPlat = hardwareMap.get(Servo.class, "left_plat");
        rightPlat = hardwareMap.get(Servo.class, "right_plat");
        
        elementServo = hardwareMap.get(Servo.class, "element_servo");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        
        spinLeft.setDirection(DcMotor.Direction.FORWARD);
        spinRight.setDirection(DcMotor.Direction.FORWARD);
        
        horizontalSlide.setDirection(DcMotor.Direction.FORWARD);

        spool.setDirection(DcMotor.Direction.FORWARD);
        spool.setTargetPosition(0);

        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        rotateServo.setPosition(rotatePower);
        elementServo.setPosition(0.25);
        grabServo.setPosition(1);

        leftPlat.setPosition(0);
        rightPlat.setPosition(1);

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
        //spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        if(gamepad2.a){
            while(gamepad2.a){
            }
            rotatePower++;
        }

        if(gamepad1.b){
            while(gamepad1.b){                
            }
            platState++;
        }

        if(gamepad2.b){
            while(gamepad2.b){
            }
            elementState++;
        }

        if (gamepad2.right_bumper){
            while(gamepad2.right_bumper){
            }
            grabPosition++;
        }
        if(gamepad1.y){
            while(gamepad1.y){
            }
            slowMode += 1;
        }
        if(gamepad2.x){
            while(gamepad2.x){
            }
            manualOverride++;
        }
        precisionRatio = (slowMode % 2) + 1;
        
        double lsx = -gamepad1.left_stick_x;
        double lsy = -gamepad1.left_stick_y;
        double rsx = gamepad1.right_stick_x;
        double rsy = gamepad1.right_stick_y;

        if(lsx < -0.75){
          minClip = -0.455;
          maxClip = 0.605;
        }else if(lsx > 0.75){
            minClip = -0.605;
            maxClip = 0.605;
        }else{
          minClip = -1;
          maxClip = 1;
        }
        
        spinLeft.setPower(feedPower/precisionRatio);
        spinRight.setPower(-feedPower/precisionRatio);
        
        spoolPosition += ((gamepad2.right_trigger - gamepad2.left_trigger)*35);
        
        if(manualOverride % 2 == 0){
            if(spoolPosition < 0){
                spoolPosition = 0;
            }else if(spoolPosition > 8900){
                spoolPosition = 8900;
            }
        }
        
        spool.setTargetPosition(spoolPosition);
        spool.setPower(1);
        
        feedPower = -gamepad1.left_trigger + gamepad1.right_trigger;

        //leftRear.setPower(Range.clip(lsy - lsx - rsx, minClip, maxClip)/precisionRatio);
        //leftFront.setPower(Range.clip(lsy + lsx - rsx, minClip, maxClip)/precisionRatio);
        //rightRear.setPower(Range.clip(lsy - lsx + rsx, -1, 1)/precisionRatio);
        //rightFront.setPower(Range.clip(lsy + lsx + rsx, minClip, maxClip)/precisionRatio);
        
        leftRear.setPower(Range.clip(lsy - rsx - lsx, minClip, maxClip)/precisionRatio);
        leftFront.setPower(Range.clip(lsy + rsx - lsx,  minClip, maxClip)/precisionRatio);
        rightRear.setPower(Range.clip(lsy + rsx + lsx, -1, 1)/precisionRatio);
        rightFront.setPower(Range.clip(lsy - rsx + lsx, minClip, maxClip)/precisionRatio);

        //leftRear.setPower(Range.clip(lsy - lsx - rsx, -1, 1)/precisionRatio);
        //leftFront.setPower(Range.clip(lsy + lsx - rsx, -1, 1)/precisionRatio);
        //rightRear.setPower(Range.clip(lsy - lsx + rsx, -1, 1)/precisionRatio);
        //rightFront.setPower(Range.clip(lsy + lsx + rsx, -1, 1)/precisionRatio);
        
        //spool.setPower(-gamepad2.left_trigger + gamepad2.right_trigger);
        
        if(grabPosition % 2 == 1){
            grabServo.setPosition(0.583);
        }else{
            grabServo.setPosition(1);
        }
        
        if(elementState % 2 == 1){
            elementServo.setPosition(0.67);
        }else{
            elementServo.setPosition(0.25);
        }
        
        if(rotatePower % 2 == 1){
            rotateServo.setPosition(1);
        }else{
            rotateServo.setPosition(0.001);
        }
        
        if(platState % 2 == 1){
            leftPlat.setPosition(1);
            rightPlat.setPosition(0);
        }else{
            leftPlat.setPosition(0);
            rightPlat.setPosition(1);
        }
        
        horizontalSlide.setPower((gamepad2.left_stick_y));

        /*
        * Code so that left stick goes forward backward and strafes
        * Right stick used to rotate
        */

        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString() + " Lift: " + spool.getCurrentPosition());
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
