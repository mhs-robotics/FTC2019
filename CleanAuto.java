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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name = "One-StoneBlue", group = "Iterative Opmode")
public class MechBotAutoDemo extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    //Declare all electronics being used
    private DcMotor leftRear;
    private DcMotor leftFront;
    private DcMotor rightRear;
    private DcMotor rightFront;
    private DcMotor spinLeft = null;
    private DcMotor spinRight = null;
    
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables.
        // The name in quotes must be the same as what is on the phone configuration
        // This is done to tell the code that the variable "leftRear" for example...
        // ...corisponds with a real motor on the robot
        leftRear = hardwareMap.get(DcMotor.class, "left_rear");
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        rightRear = hardwareMap.get(DcMotor.class, "right_rear");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        spinLeft = hardwareMap.get(DcMotor.class, "spin_left");
        spinRight = hardwareMap.get(DcMotor.class, "spin_right");

        // Set the motor direction
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        spinLeft.setDirection(DcMotor.Direction.FORWARD);
        spinRight.setDirection(DcMotor.Direction.REVERSE);
        
        //This tells all the motors with encoders to use them
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Tell the driver that initialization is complete.
        telemetry.addData("Init Complete");

        //Wait for the driver to hit play
        waitForStart();

        /*
        This is where all the coding goes
         |
         V
        */

        //Move to blocks
        //2300 is a full strafe to the blocks
        leftStrafe(1,2000);
        sleep(100);
        turnLeft(1,350);

        //Intake 1 block
        spinOn(true);
        sleep(300);
        forward(0.2,450);
        sleep(100);
        spinOn(false);
        backward(0.3, 200);

        sleep(500);
        
        //Move to other side
        turnRight(1,350);
        rightStrafe(1,700);
        sleep(200);
        backward(1,1600);

        sleep(500);
        
        //Spin Spit and Sit
        turnLeft(0.7, 1300);
        spinOn(true);
        sleep(1000);
        spinOn(false);
        turnRight(0.7, 1200);
        forward(0.7, 700);
        


        //
        //End coding here
        //

        telemetry.addData("Status", "Initialized");
    }
    
    //
    // After this comment is all the methods for movement 
    //  

    void spinOn(boolean spinon) {
        if (spinon){
            spinLeft.setPower(1);
            spinRight.setPower(1);
        } else {
            spinLeft.setPower(0);
            spinRight.setPower(0);
        }
    }
    
    void turnRight(double speed, double inches) {
        //Why???
        //speedMax/=2;
        //speedMin/=2;

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double total = leftRear.getCurrentPosition()+inches;
        double start = leftRear.getCurrentPosition();

        while(leftRear.getCurrentPosition() < total){
            leftRear.setPower(speed);
            leftFront.setPower(speed);
            rightRear.setPower(-speed);
            rightFront.setPower(-speed);

            telemenary.update();
        }
        
        leftRear.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }

    void turnLeft(double speed, double inches) {
        //Why??
        //speedMax/=2;
        //speedMin/=2;

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double total=leftRear.getCurrentPosition()+inches;
        double start=leftRear.getCurrentPosition();

        while(leftRear.getCurrentPosition()<total){
            leftRear.setPower(-speed);
            leftFront.setPower(-speed);
            rightRear.setPower(speed);
            rightFront.setPower(speed);

            telemetry.update();
        }
        
        leftRear.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }


    void rightStrafe(float speed, float degrees){

        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       
        double total=leftRear.getCurrentPosition()+degrees;
        double start=leftRear.getCurrentPosition();

        while(leftRear.getCurrentPosition() < total){
            leftRear.setPower(-speed);
            leftFront.setPower(speed);
            rightRear.setPower(speed);
            rightFront.setPower(-speed);

            telemetry.update(); 
        }
      
        leftRear.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }

    void backward(double speed, double inches){
        //choose 1 motor to do all the counting (this should stay consistent across all methods then reset that motors
        //encoders
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //declare variables for total target position and start position
        double total = leftRear.getCurrentPosition() - inches;
        double start = leftRear.getCurrentPosition();

        //this whileloop runs while the robot has not gone farther past the total
        while(leftRear.getCurrentPosition()>total){
            leftRear.setPower(-speed);
            leftFront.setPower(-speed);
            rightRear.setPower(-speed);
            rightFront.setPower(-speed);
            
            telemetry.update();

        }
        
        //Shuts off all the motors
        leftRear.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }


   void forward(double speed, double inches){
       leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

       double total=leftRear.getCurrentPosition()+inches;
       double start=leftRear.getCurrentPosition();

       while(leftRear.getCurrentPosition()<total){
            leftRear.setPower(speed);
            leftFront.setPower(speed);
            rightRear.setPower(speed);
            rightFront.setPower(speed);

            telemetry.update();
        }
       
        leftRear.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
   }

    void leftStrafe(float speed,float degrees){
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       
        double total = leftRear.getCurrentPosition() + degrees;
        double start = leftRear.getCurrentPosition();

        while(leftRear.getCurrentPosition()<total){
            leftRear.setPower(speed);
            leftFront.setPower(speed);
            rightRear.setPower(speed);
            rightFront.setPower(speed);

            telemetry.update(); 
        }
       
        leftRear.setPower(0);
        leftFront.setPower(0);
        rightRear.setPower(0);
        rightFront.setPower(0);
    }
}
