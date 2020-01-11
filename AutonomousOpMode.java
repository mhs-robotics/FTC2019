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


@Autonomous(name = "park", group = "Iterative Opmode")
public class OnlyParking extends LinearOpMode
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
        //rightFront.setTargetPosition(0);
        //leftFront.setTargetPosition(0);
        //rightRear.setTargetPosition(0);
        //leftRear.setTargetPosition(0);
        
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        waitForStart();
        
        strafeRight(5000);
        allOff();
        //forwardBackward(-10);

        //turn(true,5);
        telemetry.addData("Status", "Initialized");
    }
     void strafeRight(float inches){
        //leftFront.setTargetPosition((int)(inches * rotatePerInchStrafing)+ 90000);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setTargetPosition((int)(inches * rotatePerInchStrafing));

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setTargetPosition((int)(-inches * rotatePerInchStrafing));
        
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setTargetPosition((int)(-inches * rotatePerInchStrafing));
        
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setTargetPosition((int)(inches * rotatePerInchStrafing));

        leftFront.setPower(0.405);
        leftRear.setPower(-1);
        rightFront.setPower(-0.405);
        rightRear.setPower(1);
        
        while(rightRear.getCurrentPosition() < inches){
        }
    }
    
    void strafeLeft(float inches){
        maxPower= (float)0.605;
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setTargetPosition((int)(-inches * rotatePerInchStrafing));
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setTargetPosition((int)(inches * rotatePerInchStrafing));
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setTargetPosition((int)(inches * rotatePerInchStrafing));
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setTargetPosition((int)(-inches * rotatePerInchStrafing));
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftFront.setPower(-.405);
        leftRear.setPower(1);
        rightFront.setPower(.405);
        rightRear.setPower(-1);
        
        while(rightFront.getCurrentPosition() < inches){}
        allOff();
    }
    
    void forwardBackward(float inches){
        if(inches>(float)0){
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
        leftFront.setPower(1);
        leftRear.setPower(1);
        rightRear.setPower(1);
        rightFront.setPower(1);
        }else{
            leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setTargetPosition((int)(-inches * rotatePerInchForwardBackward));

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setTargetPosition((int)(-inches * rotatePerInchForwardBackward));

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setTargetPosition((int)(-inches * rotatePerInchForwardBackward));

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setTargetPosition((int)(-inches * rotatePerInchForwardBackward));
        leftFront.setPower(-1);
        leftRear.setPower(-1);
        rightRear.setPower(-1);
        rightFront.setPower(-1);
        
        }
        
        allOff();
    }
    void turn(boolean right, float degrees){
        

        if(right){
            
         leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setTargetPosition((int)(-1000000 * rotatePerInchForwardBackward) - 900000);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setTargetPosition((int)(-1000000 * rotatePerInchForwardBackward));

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setTargetPosition((int)(100000 * rotatePerInchForwardBackward));

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setTargetPosition((int)(100000 * rotatePerInchForwardBackward));

            allOnRight();
        }else{
             leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFront.setTargetPosition((int)(10000000 * rotatePerInchForwardBackward) + 900000);

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftRear.setTargetPosition((int)(100000000 * rotatePerInchForwardBackward));

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setTargetPosition((int)(-10000000 * rotatePerInchForwardBackward));

        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightRear.setTargetPosition((int)(-1000000 * rotatePerInchForwardBackward));

            allOnLeft();
        }
        while(rightFront.isBusy()) {
            if(right){
            if(rightFront.getCurrentPosition()>degrees){
                break;
            }
            }else{
            
            }
        }
        allOff();
    }
    
    
    void strafe(float inches){
        maxPower= (float)0.605;
        leftFront.setTargetPosition((int)(-500000 * rotatePerInchStrafing));
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftRear.setTargetPosition((int)(500000 * rotatePerInchStrafing));
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightFront.setTargetPosition((int)(500000 * rotatePerInchStrafing));
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        rightRear.setTargetPosition((int)(-500000 * rotatePerInchStrafing));
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        allOn();
        while(leftFront.isBusy() ||rightFront.isBusy()||leftRear.isBusy()||rightRear.isBusy()&& opModeIsActive()) {
        }
        allOff();
    }
    void allOnRight(){
        leftFront.setPower(-1);
        leftRear.setPower(-1);
        rightRear.setPower(1);
        rightFront.setPower(1);
    }
    void allOnLeft(){
        leftFront.setPower(1);
        leftRear.setPower(1);
        rightRear.setPower(-1);
        rightFront.setPower(-1);
    }
    void allOn(){
        leftFront.setPower(maxPower);
        leftRear.setPower(maxPower);
        rightRear.setPower(1);
        rightFront.setPower(maxPower);
    }
    void allOnForwardBack(){
        leftFront.setPower(maxPower);
        leftRear.setPower(maxPower);
        rightRear.setPower(.48);
        rightFront.setPower(.48);
    }
    
    void allOff(){
        leftFront.setPower(0);
        leftRear.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }

}
