#ifndef gyroPID_h
#define gyroPID_h

// Claw PID Variables
int distance_claw;
float Kp_claw = 0.06;
float Kd_claw = 0.035;
float Ki_claw = 0.0001;

// Task to control claw movement
task clawPID() {
    int integralActiveZone = 1200;
    float deduction = 1;
    float integralLimit = 30/(Ki_claw*deduction);
    
    float error;
    int lastError;
    
    float proportion;
    float derivative;
    float integral;
    float integral_final;
    float driveRaw;
    
    float driveDeduction;
    
    int timer = 0;
    
    while(true)
    {
        error = distance_claw-SensorValue[clawpot];
        
        proportion = Kp_claw*error;
        
        derivative = Kd_claw*(error-lastError);
        lastError = error;
        
        if(abs(error) < integralActiveZone && abs(error) > 300)
        {
            integral = integral+error;
        }
        else if (abs(error) < 300)
        {
            integral = 0;
        }
        else
        {
            integral = 0;
        }
        
        if (integral > integralLimit)
        {
            integral = integralLimit;
        }
        else if (integral < -(integralLimit))
        {
            integral = -(integralLimit);
        }
        
        integral_final = Ki_claw*integral;
        
        driveRaw = proportion+derivative+integral_final;
        
        if (driveRaw > deduction*127)
        {
            driveRaw = deduction*127;
        }
        else if (driveRaw < -deduction*127)
        {
            driveRaw = -deduction*127;
        }
        
        // End conditions (to hold against elastics)
        if(SensorValue[clawpot] > 2700 && error > 0)//SensorValue[chainpot] < 1000)
        {
            driveRaw = 25;
        }
        
        motor[claw] = driveRaw;
        
        wait1Msec(25);
    }
}

//////////////////// Chain Bar PID Task //////////////////////////////
int distance_chain;

float Kp_chain;
float Kd_chain;
float Ki_chain = 0.000037;

task chainPID()
{
    int integralActiveZone = 2500;
    float deduction = 1;
    float integralLimit = 30/(Ki_chain*deduction);
    
    float error;
    int lastError;
    
    float proportion;
    float derivative;
    float integral;
    float integral_final;
    float driveRaw;
    
    float driveDeduction;
    
    while(true)
    {
        error = distance_chain-SensorValue[chainpot];
        
        // going up
        if(error < 0)
        {
            Kp_chain = 0.062;
            Kd_chain = 0.105;
            Ki_chain = 0.000000000000000225;
        }
        // going down
        else
        {
            Kp_chain = 0.045;
            Kd_chain = 0.15;
            Ki_chain = 0.00002;
        }
        
        proportion = Kp_chain*error;
        
        derivative = Kd_chain*(error-lastError);
        lastError = error;
        
        if(abs(error) < integralActiveZone && abs(error) > 300)
        {
            integral = integral+error;
        }
        else if (abs(error) < 300)
        {
            integral = 0;
        }
        else
        {
            integral = 0;
        }
        
        if (integral > integralLimit)
        {
            integral = integralLimit;
        }
        else if (integral < -(integralLimit))
        {
            integral = -(integralLimit);
        }
        
        integral_final = Ki_chain*integral;
        
        driveRaw = proportion+derivative+integral_final;
        
        if (driveRaw > deduction*127)
        {
            driveRaw = deduction*127;
        }
        else if (driveRaw < -deduction*127)
        {
            driveRaw = -deduction*127;
        }
        
        
        if((SensorValue[chainpot] > 3200) && error > 0)
            driveRaw = 7;
        else if((SensorValue[chainpot] > 2700) &&  error > 0)
        {
            driveRaw = 40;
        }
        
        writeDebugStreamLine("%d", driveRaw);
        
        motor[chainbar] = -driveRaw;
        
        if(error < 0)
            wait1Msec(50);
        else
            wait1Msec(25);
    }
}

// Gyro + PID Functions
int gyrooutput;
int gyrocurrentvalue = 0;

void moveleftbase (int speed)
{
    motor[left1] = speed;
    motor[left2] = speed;
    motor[left3] = speed;
}

void moverightbase (int speed)
{
    motor[right1] = speed;
    motor[right2] = speed;
    motor[right3] = speed;
}

void turnleft (int speed)
{
    motor[right1] = speed;
    motor[right2] = speed;
    motor[right3] = speed;
    motor[left1] = -speed;
    motor[left2] = -speed;
    motor[left3] = -speed;
}

void stopbase()
{
    motor[right1] = 0;
    motor[right2] = 0;
    motor[right3] = 0;
    motor[left1] = 0;
    motor[left2] = 0;
    motor[left3] = 0;
}

void fixgyro()
{
    SensorFullCount[Gyro] = 36000;
}

void resetGyro (int value = 0)
{
    gyrocurrentvalue = SensorValue[Gyro] + value;
}

task GyroProcessingBlock() //This block enables users to zero out gyro value.
{
    while (true)
    {
        gyrooutput = SensorValue[Gyro] - gyrocurrentvalue;
        wait1Msec(10);
    }
}

//////////////////// PID Drive Straight Function //////////////////////////////
void PIDGyroDriveStraight (int distance, int waittime, float deduction = 1, float kP = 0.057)
{
    float Kd = 0.42;
    float Ki = 0.000051;
    
    float integralLimit = 30/(Ki*deduction);
    int integralActiveZone = 2000;
    
    float error;
    int lastError;
    
    float proportion;
    float derivative;
    float integral;
    float integral_final;
    float driveRaw;

    float Kp_G = 0.01;
    float error_drift;
    float driveDeduction;
    
    float driveLeft;
    float driveRight;
    
    int timer = 0;
    
    SensorValue[leftquad] = 0;
    SensorValue[rightquad] = 0;
    
    resetGyro();
    clearTimer(T1);
    
    while (time1[T1] < waittime)
    {
        error = distance-(SensorValue[leftquad]-SensorValue[rightquad]);
        
        proportion = kP*error;
        
        derivative = Kd*(error-lastError);
        lastError = error;
        
        if(abs(error) < integralActiveZone && abs(error) > 200)
        {
            integral = integral+error;
        }
        else if (abs(error) < 200)
        {
            integral = 0;
        }
        else
        {
            integral = 0;
        }
        
        if (integral > integralLimit)
        {
            integral = integralLimit;
        }
        else if (integral < -(integralLimit))
        {
            integral = -(integralLimit);
        }
        
        integral_final = Ki*integral;
        
        driveRaw = proportion+derivative+integral_final;
        
        if (driveRaw > deduction*127)
        {
            driveRaw = deduction*127;
        }
        else if (driveRaw < -deduction*127)
        {
            driveRaw = -deduction*127;
        }
        
        error_drift = gyrooutput;
        
        driveDeduction = Kp_G*error_drift;
        
        driveLeft = driveRaw+driveDeduction;
        driveRight = driveRaw-driveDeduction;
        
        moveleftbase(driveLeft);
        moverightbase(driveRight);
        
        wait1Msec(25);
        
        if (abs(error) < 25)
        {
            timer = 1;
            break;
        }
    }
    stopbase();
}

/////////////////// Gyro PID Function //////////////////////////////
void PIDBaseGyroTurn (int target, int waittime, float deduction = 1, float Kp = 0.138, int type = 0)
{
    float Kd = 0.557;
    float Ki = 0.00005;
    
    float gyroerror;
    int gyrolastError;
    
    float proportion;
    float derivative;
    float integral;
    float integral_final;
    float drive;
    
    float  integralLimit = 30/(Ki*deduction);
    int integralActiveZone = 500;
    
    int timer = 0;
    
    if (type == 0)
    {
        resetGyro();
    }
    else if (type == 1)
    {
        resetGyro(SensorValue[Gyro]);
    }
    
    clearTimer(T1);
    
    while (time1[T1] < waittime)
    {
        gyroerror = target-gyrooutput;       //calculate error
        
        proportion = Kp*gyroerror;
        
        derivative = Kd*(gyroerror-gyrolastError);            //calculate the derivative
        gyrolastError  = gyroerror;
        
        if(abs(gyroerror) < integralActiveZone && abs(gyroerror) > 100)
        {
            integral = integral+gyroerror;
        }
        else if (abs(gyroerror) < 100)
        {
            integral = 0;
        }
        else
        {
            integral = 0;
        }
        
        if (integral > integralLimit)
        {
            integral = integralLimit;
        }
        else if (integral < -(integralLimit))
        {
            integral = -(integralLimit);
        }
        
        integral_final = Ki*integral;
        
        drive = proportion+derivative+integral_final;
        
        if (drive > deduction*127)
        {
            drive = deduction*127;
        }
        else if (drive < -deduction*127)
        {
            drive = -deduction*127;
        }
        
        turnleft(drive);
        
        wait1Msec(25);
        
        if (abs(gyroerror) < 20)
        {
            timer = 1;
            break;
        }
    }
    stopbase();
}

// Integer to store height of cone stack
int cone = 0;

// Function to control cone stacking
void scoreCone()
{
    distance_claw = claw_closed;
    
    while(SensorValue[clawpot] < 2700)
    {}
    
    delay(100);//100
    
    // First three cones
    if(cone < 3)
    {
        distance_chain = chain_mid-100;
        
        // First cone
        if(cone == 0)
        {
            while(SensorValue[chainpot] > (chain_mid+950))
            {}
        }
        // 2 and 3
        else
        {
            while(SensorValue[chainpot] > (chain_mid + 900))
            {}
        }
        
        // Open claw
        distance_claw = claw_open;
        
        // While claw opening
        while(SensorValue[clawpot] > 2300)
        {}
        
        if(cone < 2)
            delay(100);
        else
            delay(125);
    }
    else
    {
        distance_chain = chain_mid-150;
        
        while(SensorValue[chainpot] > (chain_mid+300))
        {}
        
        delay(200);
        
        // Open claw
        distance_claw = claw_open;
        
        // While claw opening
        while(SensorValue[clawpot] > 2400)
        {}
        
        delay(150);
        
    }
    if(cone < 3)
    {
        distance_chain = chain_bottom;
        while(SensorValue[chainpot] > 3400)
        {}
    }
    cone++;
}

#endif /* gyroPID_h */
