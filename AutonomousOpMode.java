package org.firstinspires.ftc.teamcode;
/* here is a change
 */

/* This is mr. love's contribution
 */

/* Copyright (c) 2019 FIRST. All rights reserved.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to
 imitations in the disclaimer below) provided that
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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name = "Autonomous", group = "Iterative Opmode")
public class AutonomousOpMode extends LinearOpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    final static float rotatePerInchForwardBackward = 1;
    final static float rotatePerInchStrafing = 1;
    final static float rotationsPerDegreeRightLeft = 1;
    static float maxPower = (float)1;

    private DcMotor leftRear;
    private DcMotor leftFront;
    private DcMotor rightRear;
    private DcMotor rightFront;
    //private DcMotor spinRight;
    //private DcMotor spinLeft;
    //private DcMotor horizontalSlide;
    //private DcMotor spool;
    //private Servo grabServo;
    //private Servo rotateServo;
    //private Servo grabPlatform;


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightFront.setTargetPosition(0);
        leftFront.setTargetPosition(0);
        rightRear.setTargetPosition(0);
        leftRear.setTargetPosition(0);
        
        
        
        
      // spinRight = hardwareMap.get(DcMotor.class, "spin_right");
        //spinLeft = hardwareMap.get(DcMotor.class, "spin_left");

       // grabServo = hardwareMap.get(Servo.class, "grab_servo");
        //rotateServo = hardwareMap.get(Servo.class, "rotate_servo");

     //   grabPlatform=  hardwareMap.get(Servo.class, "grab_platform");

       // spool = hardwareMap.get(DcMotor.class, "spool_motor");
        //horizontalSlide = hardwareMap.get(DcMotor.class, "horizontal_slide");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
        waitForStart();
        forwardBackward((float)500);
        

    }
    

    /*feeds in a block but keeps the spinning motors running*/
    void feedIn(double power){
       // rotateServo.setPosition(/*the value that allows the robot to intake*/);
        //grabServo.setPosition(/*the value that allows the robot to intake*/);

        //spinRight.setPower(power);
        //spinLeft.setPower(power);

        //grabServo.setPosition(/*the value that locks the block*/);

    }
    /*releases block but does not retract slide or spool*/
    /*void releaseBlock(double powerSlide, double powerSpool, int slideDegrees, int spoolDegrees){

        spool.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spool.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spool.setTargetPosition(spoolDegrees);

        horizontalSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlide.setTargetPosition(slideDegrees);

        horizontalSlide.setPower(powerSlide);
        spool.setPower(powerSpool);

        while(spool.isBusy()||horizontalSlide.isBusy()){
            if(!spool.isBusy()){
                spool.setPower(0);
            }
            if(!horizontalSlide.isBusy()){
                horizontalSlide.setPower(0);
            }
        }
        spool.setPower(0);
        horizontalSlide.setPower(0);
    }*/
   
    
    void forwardBackward(float inches){
        maxPower=1;
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setTargetPosition((int)(inches * rotatePerInchForwardBackward));

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setTargetPosition((int)(inches * rotatePerInchForwardBackward));

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setTargetPosition((int)(inches * rotatePerInchForwardBackward));

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setTargetPosition((int)(inches * rotatePerInchForwardBackward));

        allOn();
        while(leftFront.isBusy() ||rightFront.isBusy()||leftRear.isBusy()||rightRear.isBusy()&& opModeIsActive()) {
        
            if(!leftFront.isBusy())
                leftFront.setPower(0);
            if(!rightFront.isBusy())
                rightFront.setPower(0);
            if(!leftRear.isBusy())
                leftRear.setPower(0);
            if(!rightRear.isBusy())
                rightRear.setPower(0);
                
        }
        allOff();
    }
    void turn(float degrees){
        maxPower=.605;
        leftFront.setTargetPosition((int)(degrees * rotationsPerDegreeRightLeft));
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRear.setTargetPosition((int)(degrees * rotationsPerDegreeRightLeft));
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        rightFront.setTargetPosition((int)(-degrees * rotationsPerDegreeRightLeft));
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightRear.setTargetPosition((int)(-degrees * rotationsPerDegreeRightLeft));
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        allOn();
        while(leftFront.isBusy() && opModeIsActive()) {
            if(!leftFront.isBusy())
                leftFront.setPower(0);
            if(!rightFront.isBusy())
                rightFront.setPower(0);
            if(!leftRear.isBusy())
                leftRear.setPower(0);
            if(!rightRear.isBusy())
                rightRear.setPower(0);
        }
        allOff();
    }
    void strafe(float inches){
        maxPower=.605;
        leftFront.setTargetPosition((int)(inches * rotatePerInchStrafing));
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRear.setTargetPosition((int)(-inches * rotatePerInchStrafing));
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFront.setTargetPosition((int)(inches * rotatePerInchStrafing));
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFront.setTargetPosition((int)(inches * rotatePerInchStrafing));
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        allOn();
        while(leftFront.isBusy() && opModeIsActive()) {
            if(!leftFront.isBusy())
                leftFront.setPower(0);
            if(!rightFront.isBusy())
                rightFront.setPower(0);
            if(!leftRear.isBusy())
                leftRear.setPower(0);
            if(!rightRear.isBusy())
                rightRear.setPower(0);
        }
        allOff();
    }
    void allOn(){
        leftFront.setPower(maxPower);
        leftRear.setPower(maxPower);
        rightRear.setPower(1);
        rightFront.setPower(maxPower);
    }
    void allOff(){
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }

}
