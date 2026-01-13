
#include "3dmath_helper.h"


class Madgwick_Filter;

// ----------------------------------------------------------------------------
//
//  MADGWICK FILTER CLASS DEFINITION
//
// ----------------------------------------------------------------------------


class Madgwick_Filter {
private:
    // counter variable
    double c;
    const int sizeFloat = sizeof(float);
    const int sizeVectorFloat = sizeof(VectorFloat);
    const int sizeQuaternion = sizeof(Quaternion);

    const float AV_FACTOR = 939.6507;

    // parameters for Madgwick Filter
    float alpha, beta, gamma, zeta, stepSize;
    float* vals[5] = {&alpha, &beta, &gamma, &zeta, &stepSize};
    float del_t;
    
    Quaternion wOriP,           *wop = &wOriP;
    #if USE_GRAVITY
        VectorFloat gravF,      *gF = &gravF;
        Quaternion gravGrad,    *gg = &gravGrad;
    #endif
    #if USE_MAG_NORTH
        VectorFloat magNF,      *mNF = &magNF;
        Quaternion magNGrad,    *mng = &magNGrad;
    #endif
    Quaternion gradientDir;
    
    // gyro compensation
    Quaternion angularVelBias, modAngularVel;

    // parameters for main2()
    Quaternion gradientQ;


    /*functions corner*/
#if USE_GRAVITY
    void gravityFunction(VectorFloat* gF, Quaternion* q, VectorFloat* a) {
        gF->x = 2*(q->x*q->z - q->w*q->y)    -    a->x;
        gF->y = 2*(q->w*q->x - q->y*q->z)    -    a->y;
        gF->z = 2*(0.5 - q->x*q->x - q->y*q->y) - a->z;
    }

    void gravityGradient(VectorFloat* aS) {
        aS->normalize();
        //gravityFunction(gF, wop, aS);

        gF->x = 2*(wop->x*wop->z - wop->w*wop->y)    -    aS->x;
        gF->y = 2*(wop->w*wop->x - wop->y*wop->z)    -    aS->y;
        gF->z = 2*(0.5 - wop->x*wop->x - wop->y*wop->y) - aS->z;
        
        //Serial.println("in gravityGradient():");
        gg->w = 2*(         -wop->y*gF->x + wop->x*gF->y          );
        gg->x = 2*(  wop->z*gF->x + wop->w*gF->y - 2*wop->x*gF->z );
        gg->y = 2*( -wop->w*gF->x + wop->z*gF->y - 2*wop->y*gF->z );
        gg->z = 2*(          wop->x*gF->x + wop->y*gF->y          );
        //Serial.print("gg:\t"); gg.print(); Serial.println();
    }
#endif

#if USE_MAG_NORTH
    void magNFunction(VectorFloat* mNF, Quaternion* q, VectorFloat* b, VectorFloat* m) {
        mNF->x = 2*b->x*( 0.5 - q->y*q->y - q->z*q->z )   +   2*b->z*( q->x*q->z - q->w*q->y ) - m->x;
        mNF->y = 2*b->x*( q->x*q->y - q->w*q->z )      +      2*b->z*( q->w*q->x + q->y*q->z ) - m->y;
        mNF->z = 2*b->x*( q->w*q->y + q->x*q->z )   +   2*b->z*( 0.5 - q->x*q->x - q->y*q->y ) - m->z;
    }
    
    void magNorthGradient(VectorFloat* mS) {
        static VectorFloat magB = VectorFloat(), *mB = &magB;
        static Quaternion conjug, *wopc = &conjug;
        
        conjug = wop->getConjugate();
        mS->rotate( wopc );
        mS->normalize();
        mB->x = sqrt(mS->x*mS->x + mS->y*mS->y);
        mB->z = mS->z;

        //magNFunction(mnf, wop, mB, mS);

        mNF->x = 2*mB->x*( 0.5 - wop->y*wop->y - wop->z*wop->z )   +   2*mB->z*( wop->x*wop->z - wop->w*wop->y ) - mS->x;
        mNF->y = 2*mB->x*( wop->x*wop->y - wop->w*wop->z )      +      2*mB->z*( wop->w*wop->x + wop->y*wop->z ) - mS->y;
        mNF->z = 2*mB->x*( wop->w*wop->y + wop->x*wop->z )   +   2*mB->z*( 0.5 - wop->x*wop->x - wop->y*wop->y ) - mS->z;

        mng->w = 2*(        -mB->z*wop->y*mNF->x        +      (-mB->x*wop->z + mB->z*wop->x)*mNF->y       +       mB->x*wop->y*mNF->z        );
        mng->x = 2*(         mB->z*wop->z*mNF->x        +       (mB->x*wop->y + mB->z*wop->w)*mNF->y + (mB->x*wop->z - 2*mB->z*wop->x)*mNF->z );
        mng->y = 2*( (-2*mB->x*wop->y - mB->z*wop->w)*mNF->x +  (mB->x*wop->x + mB->z*wop->z)*mNF->y + (mB->x*wop->w - 2*mB->z*wop->y)*mNF->z );
        mng->z = 2*( (-2*mB->x*wop->z + mB->z*wop->x)*mNF->x + (-mB->x*wop->w + mB->z*wop->y)*mNF->y       +       mB->x*wop->x*mNF->z        );
    }
#endif

    void findGradientDir(VectorFloat* accelSensor, VectorFloat* magSensor) {
        gradientDir = {0,0,0,0};
            
        #if USE_GRAVITY
            gravityGradient(accelSensor);
            gradientDir = gradientDir + gravGrad;
        #endif
        #if USE_MAG_NORTH
            magNorthGradient(magSensor);
            gradientDir = gradientDir + magNGrad;
        #endif
        
        gradientDir.normalize();
        //Serial.print("gradD:\t");  gradientDir.print();    Serial.println();
    }

    /*  gyro bias compensation  */
    void findGyroBias() {
        angularVelBias = angularVelBias + wOriP.getProduct(gradientDir)*(2*zeta*del_t);
        //Serial.print("angularVelBias:\t");  angularVelBias.print();    Serial.println();
    }

    // this function adds error data from the magnetometer and/or accelerometer 
    void findSensorFusionQ() {
        findGyroBias();
        //angularVelBias.print(); Serial.println();
        modAngularVel = modAngularVel - angularVelBias;

        // using the previous orientation to come up with the new one
        wOriN = wOriP.getProduct(angularVelBias)*0.5f;
        wOriN = wOriN + modAngularVel*del_t;

        wOriN.normalize();
        //Serial.print("wOriN:\t");   wOriN.print();  Serial.println();
    }
            
    /*functions part 2: featuring the MPU6050's DMP*/
    float calcBeta(float avMagnitude) {
        #define BETA_AVERAGING_METHOD 3
        
        #if BETA_AVERAGING_METHOD == 1
            return beta;
            
        #elif BETA_AVERAGING_METHOD == 2
            static float sum = 0;
            sum += avMagnitude;
            return sum/c;
        
        #elif BETA_AVERAGING_METHOD == 3
            // exponential moving average
            
                // change this to change the rate of decay
            const static float alpha = 0.005; // 0.002
            static float betaN = 0, betaP = 1;
            
            betaN = alpha*avMagnitude + ( 1 - alpha )*betaP;
            memcpy(&betaP, &betaN, sizeFloat);
            return betaN;
            
        #endif
    }
    float calcStepSize(float avMagnitude) { return alpha*avMagnitude*del_t; }
    float calcWeight(float stepSize) { return beta/( beta + stepSize/del_t ); }
    
public:
    
    Quaternion wOriN, *won = &wOriN;
    
    Madgwick_Filter(float a, float b, float z, float dt) {
        c = 0;
        alpha = a;
        beta = b; 
        zeta = z;
        del_t = dt;  // in seconds
        
        angularVelBias = {0,0,0,0};
        modAngularVel = {0,0,0,0};
        gradientQ = Quaternion();
        wOriN = Quaternion();
    }
    void loadOri(Quaternion* q) { wOriP = *q; }
    
    void printParameters(uint8_t sf = 3) {
        static uint8_t i;

        Serial.print("\n\nMF iter. ");  Serial.println(c, 0);   Serial.print("[a,b,g,z,u] = [");
        for (i = 0; i < 5-1; ++i) {
            Serial.print(*vals[i], sf);     Serial.print("\t");
        }   Serial.print(*vals[4], sf);     Serial.print("]");

//            for (i = 0; i < 5-1; ++i) {
//                Serial.print(*vals[i], sf);     Serial.print("\t");
//            }   Serial.print(*vals[4], sf);     Serial.print("]");
    }
    
    /*main aggregator for function calculations*/
//        void main(
//            #if USE_GRAVITY 
//                VectorFloat* accelSensor
//            #endif
//            #if USE_GRAVITY & USE_MAG_NORTH
//                , 
//            #endif
//            #if USE_MAG_NORTH 
//                VectorFloat* magSensor
//            #endif
//            )
//        {
//            /*work in progress*/
//            
//            memcpy(&wOriP, &wOriN, sizeQuaternion);
//            //memswap(wop, won);
//            ++c;
//            Serial.println(c, 0);
//            
//            Serial.println("findGradientDir()");
//            findGradientDir(magSensor);
//            Serial.println("findSensorFusionQ()");
//            findSensorFusionQ();
//        }
    
    void DMP_Main(
            VectorInt16* accelSensor,
            VectorFloat* magU,
            Quaternion* dmpQ,
            float avMag)
    {
        //#define MF_RECORD_TIME

        static VectorFloat accelU = VectorFloat();            
        
        #ifdef MF_RECORD_TIME
            static long time1 = 0, time2 = 0;
            time1 = micros();
        #endif
        
        memcpy(wop, won, sizeQuaternion);
        //memswap(wop, won);

        #if USE_GRAVITY
            accelU = (VectorFloat)(*accelSensor);
            accelU.normalize();
        #endif
        
        
        ++c;
        avMag /= AV_FACTOR;
        beta = calcBeta(avMag);
        stepSize = calcStepSize(avMag);
        gamma = calcWeight(stepSize);
        
//            printParameters();
        findGradientDir(&accelU, magU);
        wOriN = ( wOriP - gradientDir*stepSize )*gamma + (*dmpQ)*( 1.f - gamma );
        won->normalize();
        
        // Serial.print("\ngradQ:\t");     gradientQ.print(2);
        // Serial.print("\ndmpQ:\t");      dmpQ->print(2);
        // Serial.print("\nmfQ:\t");       wOriN.print(2);
        #ifdef MF_RECORD_TIME
            time2 = micros();
            Serial.print("\n\tmfTime = ");  Serial.print(time2 - time1);    Serial.println(" us");
        #endif
    }
};
