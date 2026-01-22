#ifndef __MADGWICK_FILTER_H__
#define __MADGWICK_FILTER_H__


#include "helper_3dmath.h"

// #define CHANGES 1


// ----------------------------------------------------------------------------
// 
//  MADGWICK FILTER CONFIGURATION
// 
// ----------------------------------------------------------------------------

#define USE_GRAVITY 1
#define USE_MAG_NORTH 1

class Madgwick_Filter {
    
private:

    // counter variable
    // double c;
    const int sizeFloat         = sizeof(float);
    const int sizeVectorFloat   = sizeof(VectorFloat);
    const int sizeQuaternion    = sizeof(Quaternion);

    const float AV_FACTOR = 939.6507f;

    // parameters for Madgwick Filter
    float alpha, beta, gamma, zeta, stepSize;
    float* vals[5] = {&alpha, &beta, &gamma, &zeta, &stepSize};
    float del_t;
    
    Quaternion wOriP,           *q_p = &wOriP;

    #if USE_GRAVITY
    VectorFloat     gravF;
    VectorFloat*    gF  = &gravF;

    Quaternion      gravGrad;
    Quaternion*     gg = &gravGrad;
    #endif

    #if USE_MAG_NORTH
    VectorFloat     magNF;
    VectorFloat*    mNF = &magNF;

    Quaternion      magNGrad;
    Quaternion*     mng = &magNGrad;
    #endif
    
    // gyro compensation
    Quaternion angularVelBias, modAngularVel;


    /* functions corner */
    #if USE_GRAVITY
    void gravityFunction(
        VectorFloat*    gF,
        Quaternion*     q,
        const VectorFloat*    a)
    {
        gF->x   = 2*(      q->x*q->z - q->w*q->y) - a->x;
        gF->y   = 2*(      q->w*q->x - q->y*q->z) - a->y;
        gF->z   = 2*(0.5 - q->x*q->x - q->y*q->y) - a->z;
    }

    /**
     * @brief 
     * 
     * @param[in] accel 
     */
    void compute_gravity_gradient(VectorFloat* accel) {

        accel->normalize();
        // gravityFunction(gF, q_p, accel);

        gF->x = 2*(      q_p->x*q_p->z - q_p->w*q_p->y) - accel->x;
        gF->y = 2*(      q_p->w*q_p->x - q_p->y*q_p->z) - accel->y;
        gF->z = 2*(0.5 - q_p->x*q_p->x - q_p->y*q_p->y) - accel->z;

        gg->w = 2*( -q_p->y*gF->x + q_p->x*gF->y                  );
        gg->x = 2*(  q_p->z*gF->x + q_p->w*gF->y - 2*q_p->x*gF->z );
        gg->y = 2*( -q_p->w*gF->x + q_p->z*gF->y - 2*q_p->y*gF->z );
        gg->z = 2*(  q_p->x*gF->x + q_p->y*gF->y                  );


        // Debugging prints
        // Serial.println("in compute_gravity_gradient():");
        // Serial.print("gg:\t"); gg->print(); Serial.println();
    }
    #endif

    #if USE_MAG_NORTH
    void magNFunction(
            VectorFloat*    mNF,
            Quaternion*     q,
            VectorFloat*    b,
            VectorFloat*    m)
    {
        mNF->x =    2*b->x*( 0.5 - q->y*q->y - q->z*q->z )
                +   2*b->z*(       q->x*q->z - q->w*q->y )
                -   m->x;
        mNF->y =    2*b->x*(       q->x*q->y - q->w*q->z )
                +   2*b->z*(       q->w*q->x + q->y*q->z )
                -   m->y;
        mNF->z =    2*b->x*(       q->w*q->y + q->x*q->z )
                +   2*b->z*( 0.5 - q->x*q->x - q->y*q->y )
                -   m->z;
    }
    
    /**
     * @brief 
     * 
     * @param mS 
     * 
     * $$q_w$$
     */
    void magNorthGradient(VectorFloat* mS) {

        VectorFloat mB;

        mS->normalize();
        mB.x   = sqrt(mS->x*mS->x + mS->y*mS->y);
        mB.z   = mS->z;
        // magNFunction(mnf, q_p, mB, mS);

        mNF->x =    2*mB.x*( 0.5 - q_p->y*q_p->y - q_p->z*q_p->z )  
                 +  2*mB.z*(       q_p->x*q_p->z - q_p->w*q_p->y )
                 -  mS->x;
        mNF->y =    2*mB.x*(       q_p->x*q_p->y - q_p->w*q_p->z )
                 +  2*mB.z*(       q_p->w*q_p->x + q_p->y*q_p->z )
                 -  mS->y;
        mNF->z =    2*mB.x*(       q_p->w*q_p->y + q_p->x*q_p->z )
                 +  2*mB.z*( 0.5 - q_p->x*q_p->x - q_p->y*q_p->y )
                 -  mS->z;

        // compute gradient
        /**
         * @note confirmed analytically
         */
        mng->w = 2*(     -mB.z*q_p->y*mNF->x        
                    + (  -mB.x*q_p->z +   mB.z*q_p->x)*mNF->y
                    +     mB.x*q_p->y*mNF->z                    );
        mng->x = 2*(      mB.z*q_p->z*mNF->x
                    + (   mB.x*q_p->y +   mB.z*q_p->w)*mNF->y
                    + (   mB.x*q_p->z - 2*mB.z*q_p->x)*mNF->z   );
        mng->y = 2*(  (-2*mB.x*q_p->y -   mB.z*q_p->w)*mNF->x
                    + (   mB.x*q_p->x +   mB.z*q_p->z)*mNF->y
                    + (   mB.x*q_p->w - 2*mB.z*q_p->y)*mNF->z   );
        mng->z = 2*(  (-2*mB.x*q_p->z +   mB.z*q_p->x)*mNF->x
                    + (  -mB.x*q_p->w +   mB.z*q_p->y)*mNF->y
                    +     mB.x*q_p->x*mNF->z                    );


        // Debugging prints
        // Serial.println("in magNorthGradient():");
        // Serial.print("gg:\t"); gg->print(); Serial.println();
    }
    #endif

    /**
     * @brief Computes the gradient vector based on accelerometer and magnetometer sensor data
     * 
     * @param[in] accelSensor VectorFloat ptr representing accelerometer sensor data
     * @param[in] magSensor VectorFloat ptr representing magnetometer sensor data
     * @param[out] gradientDir Quaternion ptr where the computed gradient direction will be stored
     */
    void compute_gradient_vector(
            VectorFloat*    accelSensor,
            VectorFloat*    magSensor,
            Quaternion*     gradientDir)
    {
        *gradientDir = {0,0,0,0};
            
        #if USE_GRAVITY
            compute_gravity_gradient(accelSensor);
            *gradientDir = *gradientDir + gravGrad;
        #endif
        #if USE_MAG_NORTH
            magNorthGradient(magSensor);
            *gradientDir = *gradientDir + magNGrad;
        #endif
        
        gradientDir->normalize();
        // Serial.print("gradD:\t");  gradientDir.print();    Serial.println();
    }

            
    /* functions part 2: featuring the MPU6050's DMP */
    /**
     * @brief Calculates the beta parameter using the specified averaging method
     * 
     * @details a good way of gating changes in orientation is by relating it to the magnitude of the angular velocity. When done this way, hard biases and other magnetic disturbances have less of an effect on the orientation estimate.
     * 
     * @note if `BETA_AVERAGING_METHOD` is set to 1, beta is constant. 
     *  in practice, the best results have been obtained using method 3
     * 
     * @param rot_rate_mag The magnitude of the angular velocity
     * @return float The calculated beta value
     * 
     * @see `BETA_AVERAGING_METHOD`
     */
    float calcBeta(float rot_rate_mag) {

    #define BETA_AVERAGING_METHOD 3
    
    // constant beta
    #if BETA_AVERAGING_METHOD == 1
        return beta;
        
    // simple moving average
    #elif BETA_AVERAGING_METHOD == 2
        static float sum = 0;
        sum += rot_rate_mag;
        return sum/c;
    

    // exponential moving average
    #elif BETA_AVERAGING_METHOD == 3
        
        /**
         * @brief change this to change the rate of decay
         * @note alpha should be in the range [0,1]
         * if alpha is closer to 1, more weight is given to the most recent sample
         * if alpha is closer to 0, more weight is given to older samples
         */
        #define ALPHA_UPDATE  0.2f
        static float betaP  = 1;
        float betaN         = 0;
        
        betaN = ALPHA_UPDATE*rot_rate_mag + ( 1 - ALPHA_UPDATE )*betaP;
        
        betaP = betaN;
        return betaN;
        
    #endif

    }


    /**
     * @brief Calculates the step size based on the magnitude of the angular velocity
     * 
     * @param rot_rate_mag The magnitude of the angular velocity
     * @return float The calculated step size
     */
    inline float calcStepSize(float rot_rate_mag) {
        return alpha*rot_rate_mag*del_t;
    }

    /**
     * @brief Calculates the weight for the Madgwick filter update
     * 
     * @note known in the literature as the `gamma` parameter
     * 
     * @param stepSize The calculated step size
     * @return float The calculated weight
     */
    inline float calcWeight(float stepSize) {
        return beta/( beta + stepSize/del_t );
    }
    
public:
    
    // Quaternion wOriN, *won = &wOriN;
    
    Madgwick_Filter(float a, float b, float z, float dt) {
        alpha   = a;
        beta    = b; 
        zeta    = z;
        del_t   = dt;  // in seconds
        
        angularVelBias  = {0,0,0,0};
        modAngularVel   = {0,0,0,0};
        // wOriN           = Quaternion();
        wOriP           = Quaternion();
    }

    void loadOri(Quaternion* q) {
        wOriP = *q;
    }
    
    void printParameters(uint8_t places = 3) {

        static uint8_t i;

        Serial.print("\n\nMF iter. ");
        Serial.print("[a,b,g,z,u] = [");

        for (i = 0; i < 5-1; ++i) {
            Serial.print(*vals[i], places);     Serial.print("\t");
        }

        Serial.print(*vals[4], places);     Serial.print("]");

        // for (i = 0; i < 5-1; ++i) {
            // Serial.print(*vals[i], places);     Serial.print("\t");
        // }   
        // Serial.print(*vals[4], places);     Serial.print("]");
    }
    
    #ifndef CHANGES
    void update(
            VectorInt16*    accelSensor,
            VectorFloat*    magU,
            Quaternion*     won,
            VectorFloat*    angular_velocity)
    {

        VectorFloat accelU = VectorFloat();
        
        Quaternion q_gradient = {0,0,0,0};

        // equation 3.30 from Madgwick's thesis
        // this will store the sum total of all the estimated changes in orientation at time t
        Quaternion q_dot = {0,0,0,0};

        // angular velocity as quaternion
        // @todo this parameter should be in place of the `angular_velocity` parameter
        Quaternion q_av = Quaternion(
                0,
                angular_velocity->x,
                angular_velocity->y,
                angular_velocity->z);

        float rot_vel_mag = q_av.getMagnitude();

        // -----------------------------------------------------------

    #if USE_GRAVITY
        accelU = (VectorFloat)(*accelSensor);
        accelU.normalize();
    #endif        
        
    
        /**
         * @brief compute a new `q_gradient` that points in the direction of the
         *  fastest change to reduce the error between measured and estimated
         * @note this is equation (3.21) from Madgwick's thesis
         */
        compute_gradient_vector(&accelU, magU, &q_gradient);


        /**
         * @brief update beta based on the rate of the angular velocity
         * @note this is equation (3.33) from Madgwick's thesis
         * 
         */
        beta = 0.5*sqrt(3) * calcBeta(rot_vel_mag);

        // adds angular velocity direction based on...
        q_dot = q_p->getProduct(q_av) * 0.5;    // 
        q_dot = q_dot  -  q_gradient * beta;

        // integrate to yield quaternion
        *won = wOriP  +  q_dot * del_t;

        won->normalize();
        
        // update previous orientation
        wOriP = *won;

    }

    #else

    void update(
            VectorInt16*    accelSensor,
            VectorFloat*    magU,
            Quaternion*     won,
            VectorFloat*    angular_velocity)
    {
        VectorFloat accelU = VectorFloat();
        
        Quaternion q_gradient = {0,0,0,0};

        // equation 3.30 from Madgwick's thesis
        // this will store the sum total of all the estimated changes in orientation at time t
        Quaternion q_dot = {0,0,0,0};

        // angular velocity as quaternion
        // @todo this parameter should be in place of the `angular_velocity` parameter
        Quaternion q_av = Quaternion(
                0,
                angular_velocity->x,
                angular_velocity->y,
                angular_velocity->z);

        float rot_vel_mag = q_av.getMagnitude();

        // -----------------------------------------------------------

    #if USE_GRAVITY
        accelU = (VectorFloat)(*accelSensor);
        accelU.normalize();
    #endif
        
    
        /**
         * @brief compute a new `q_gradient` that points in the direction of the
         *  fastest change to reduce the error between measured and estimated
         * @note this is equation (3.21) from Madgwick's thesis
         */
        compute_gradient_vector(&accelU, magU, &q_gradient);


        /**
         * @brief update beta based on the rate of the angular velocity
         * @note this is equation (3.33) from Madgwick's thesis
         * 
         */
        beta        = 0.5*sqrt(3) * calcBeta(rot_vel_mag);
        stepSize    = calcStepSize(rot_vel_mag);
        gamma       = calcWeight(stepSize);

        *won   =     ( wOriP - q_gradient*stepSize )*gamma
                  +   (wOriP)*( 1.f - gamma );

        // adds angular velocity direction based on...
        // q_dot = q_p->getProduct(q_av) * 0.5;    // 
        // q_dot = q_dot  -  q_gradient * beta;

        // // integrate to yield quaternion
        // *won = wOriP  +  q_dot * del_t;

        won->normalize();
        
        // update previous orientation
        wOriP = *won;

    }
    #endif // CHANGES
};

#endif /* __MADGWICK_FILTER_H__ */
