#ifndef __MADGWICK_FILTER_H__
#define __MADGWICK_FILTER_H__


#include "helper_3dmath.h"

// #define CHANGES 1


// ----------------------------------------------------------------------------
// 
//  MADGWICK FILTER CONFIGURATION
// 
// ----------------------------------------------------------------------------


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
     * @brief described in equations 
     * 
     * @param[in] accel 
     * @param[in] q_p 
     * @param[out] gg
     * 
     */
    void compute_gravity_gradient(
            Quaternion*     q_p, 
            VectorFloat*    accel,
            Quaternion*     gg) 
    {

        VectorFloat gF;

        accel->normalize();
        // gravityFunction(gF, q_p, accel);

        gF.x    = 2*(      q_p->x*q_p->z - q_p->w*q_p->y) - accel->x;
        gF.y    = 2*(      q_p->w*q_p->x - q_p->y*q_p->z) - accel->y;
        gF.z    = 2*(0.5 - q_p->x*q_p->x - q_p->y*q_p->y) - accel->z;

        gg->w   = 2*( -q_p->y*gF.x + q_p->x*gF.y                 );
        gg->x   = 2*(  q_p->z*gF.x + q_p->w*gF.y - 2*q_p->x*gF.z );
        gg->y   = 2*( -q_p->w*gF.x + q_p->z*gF.y - 2*q_p->y*gF.z );
        gg->z   = 2*(  q_p->x*gF.x + q_p->y*gF.y                 );


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
     * @param mag 
     * 
     */
    void compute_magNorth_gradient(
            Quaternion*     q_p,
            VectorFloat*    mag, 
            Quaternion*     mng) 
    {
        VectorFloat mNF;

        VectorFloat mB;

        mag->normalize();
        mB.x   = sqrt(mag->x*mag->x + mag->y*mag->y);
        mB.z   = mag->z;
        // magNFunction(mnf, q_p, mB, mag);

        mNF.x =    2*mB.x*( 0.5 - q_p->y*q_p->y - q_p->z*q_p->z )  
                +  2*mB.z*(       q_p->x*q_p->z - q_p->w*q_p->y )
                -    mag->x;
        mNF.y =    2*mB.x*(       q_p->x*q_p->y - q_p->w*q_p->z )
                +  2*mB.z*(       q_p->w*q_p->x + q_p->y*q_p->z )
                -    mag->y;
        mNF.z =    2*mB.x*(       q_p->w*q_p->y + q_p->x*q_p->z )
                +  2*mB.z*( 0.5 - q_p->x*q_p->x - q_p->y*q_p->y )
                -    mag->z;


        /**
         * @note confirmed analytically
         */
        mng->w = 2*(     -mB.z*q_p->y*mNF.x
                    + (  -mB.x*q_p->z +   mB.z*q_p->x)*mNF.y
                    +     mB.x*q_p->y*mNF.z                    );
        mng->x = 2*(      mB.z*q_p->z*mNF.x
                    + (   mB.x*q_p->y +   mB.z*q_p->w)*mNF.y
                    + (   mB.x*q_p->z - 2*mB.z*q_p->x)*mNF.z   );
        mng->y = 2*(  (-2*mB.x*q_p->y -   mB.z*q_p->w)*mNF.x
                    + (   mB.x*q_p->x +   mB.z*q_p->z)*mNF.y
                    + (   mB.x*q_p->w - 2*mB.z*q_p->y)*mNF.z   );
        mng->z = 2*(  (-2*mB.x*q_p->z +   mB.z*q_p->x)*mNF.x
                    + (  -mB.x*q_p->w +   mB.z*q_p->y)*mNF.y
                    +     mB.x*q_p->x*mNF.z                    );


        // Debugging prints
        // Serial.println("in compute_magNorth_gradient():");
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
            Quaternion*     q_p,
            VectorFloat*    accelSensor,
            VectorFloat*    magSensor,
            Quaternion*     gradientDir)
    {
        *gradientDir = {0,0,0,0};

        #if USE_GRAVITY
            Quaternion gravGrad_local;
            compute_gravity_gradient(q_p, accelSensor, &gravGrad_local);
            *gradientDir = *gradientDir + gravGrad_local;
        #endif
        
        #if USE_MAG_NORTH
            Quaternion magNGrad_local;
            compute_magNorth_gradient(q_p, magSensor, &magNGrad_local);
            *gradientDir = *gradientDir + magNGrad_local;
        #endif
        
        gradientDir->normalize();
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

    #define BETA_AVERAGING_METHOD 1
    
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
    
    Madgwick_Filter(float a, float b, float z, float dt) {
        alpha   = a;
        beta    = b; 
        zeta    = z;
        del_t   = dt;  // in seconds
        
        angularVelBias  = {0,0,0,0};
        modAngularVel   = {0,0,0,0};
        // wOriP           = Quaternion();
    }
    
    #ifndef CHANGES
    void update(
            VectorFloat*    accelSensor,
            VectorFloat*    magU,
            VectorFloat*    angular_velocity,
            Quaternion*     q_n)
    {

        static Quaternion wOriP = Quaternion();
        static Quaternion* q_p = &wOriP;

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

        accelU = (VectorFloat)(*accelSensor);
        accelU.normalize();
        
    
        /**
         * @brief compute a new `q_gradient` that points in the direction of the
         *  fastest change to reduce the error between measured and estimated
         * @note this is equation (3.21) from Madgwick's thesis
         */
        compute_gradient_vector(q_p, &accelU, magU, &q_gradient);


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
        *q_n = wOriP  +  q_dot * del_t;

        q_n->normalize();
        
        // update previous orientation
        wOriP = *q_n;

    }

    #else

    void update(
            VectorFloat*    accelSensor,
            VectorFloat*    magU,
            VectorFloat*    angular_velocity,
            Quaternion*     q_n)
    {

        static Quaternion wOriP = Quaternion();
        static Quaternion* q_p = &wOriP;

        VectorFloat accelU = VectorFloat();
        
        // gradient direction
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
        accelSensor->normalize();
    #endif
        
    
        /**
         * @brief compute a new `q_gradient` that points in the direction of the
         *  fastest change to reduce the error between measured and estimated
         * @note this is equation (3.21) from Madgwick's thesis
         */
        compute_gradient_vector(q_p, &accelU, magU, &q_gradient);


        /**
         * @brief update beta based on the rate of the angular velocity
         * @note this is equation (3.33) from Madgwick's thesis
         * 
         */
        beta        = 0.5*sqrt(3) * calcBeta(rot_vel_mag);
        stepSize    = calcStepSize(rot_vel_mag);
        gamma       = calcWeight(stepSize);

        *q_n   =     ( wOriP - q_gradient*stepSize )*gamma
                  +   (wOriP)*( 1.f - gamma );

        q_n->normalize();
        
        // update previous orientation
        wOriP = *q_n;

    }
    #endif // CHANGES
};

#endif /* __MADGWICK_FILTER_H__ */
