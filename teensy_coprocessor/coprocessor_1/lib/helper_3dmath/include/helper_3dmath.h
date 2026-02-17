// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class, 3D
// math helper
// 6/5/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at:
//      https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2012-06-05 - add 3D math helper file to DMP6 example sketch

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _HELPER_3DMATH_H_
#define _HELPER_3DMATH_H_

#include <Arduino.h>

//class QuaternionDouble;
//class VectorInt16;


// Quaternion crossMult(VectorInt16*, VectorInt16*);
// void getQuaternion(Quaternion*, VectorInt16*, VectorInt16*);

class Quaternion {
public:
    float w;
    float x;
    float y;
    float z;
    
    Quaternion() {
        w = 1.0f;
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }
    
    Quaternion(float nw, float nx, float ny, float nz) {
        w = nw;
        x = nx;
        y = ny;
        z = nz;
    }

    Quaternion getProduct(const Quaternion q) {

        // Quaternion multiplication is defined by:
        //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
        //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
        //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
        //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2)

        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,  // new w
            w*q.x + x*q.w + y*q.z - z*q.y,  // new x
            w*q.y - x*q.z + y*q.w + z*q.x,  // new y
            w*q.z + x*q.y - y*q.x + z*q.w   // new z
        );
    }

    Quaternion getConjugate() {
        return Quaternion(w, -x, -y, -z);
    }
    
    float getMagnitude() {
        return sqrt(w*w + x*x + y*y + z*z);
    }
    
    void normalize() {
        float m = getMagnitude();
        //if (m < 0.0001) return;
        w /= m;
        x /= m;
        y /= m;
        z /= m;
    }
    
    Quaternion getNormalized() {
        Quaternion r(w, x, y, z);
        r.normalize();
        return r;
    }
    
    Quaternion operator+(const Quaternion &a) {
        return Quaternion( w+a.w, x+a.x, y+a.y, z+a.z );
    }
    Quaternion operator-(const Quaternion &a) {
        return Quaternion( w-a.w, x-a.x, y-a.y, z-a.z );
    }
    Quaternion operator*(const float &a) {
        return Quaternion( a*w, a*x, a*y, a*z );
}
    Quaternion operator/(const float &a) {
        return Quaternion( w/a, x/a, y/a, z/a );
    }
    
    
    // Quaternion axisAngle() {
    //     Quaternion axisAngle(0, x, y, z);
    //     float uMag = axisAngle.getMagnitude();

    //     axisAngle.normalize();
    //     axisAngle.w = 2 * atan2(uMag, w) * (180.0f/PI);
    //     return axisAngle;
    // }
    
    Quaternion print(uint8_t sigfig = 3) {
        Serial.print(w, sigfig); Serial.print("\t");
        Serial.print(x, sigfig); Serial.print("\t");
        Serial.print(y, sigfig); Serial.print("\t");
        Serial.print(z, sigfig);
        return *this;
    }
};



class VectorInt16 {

public:

    int16_t x;
    int16_t y;
    int16_t z;

    VectorInt16() {
        x = 0;
        y = 0;
        z = 0;
    }
    
    VectorInt16(int16_t nx, int16_t ny, int16_t nz) {
        x = nx;
        y = ny;
        z = nz;
    }

    float getMagnitude() {
        return sqrt(x*x + y*y + z*z);
    }

    void normalize() {
        float m = getMagnitude();
        // if (m < 0.0001) return;
        x /= m;
        y /= m;
        z /= m;
    }
    
    VectorInt16 getNormalized() {
        VectorInt16 r(x, y, z);
        r.normalize();
        return r;
    }
    
    /**
     * @brief Rotates this vector by the given quaternion
     * 
     * @param[in] q Quaternion to rotate by
     * @param[out] this Vector is rotated in place
     * 
     * @see http://www.cprogramming.com/tutorial/3d/quaternions.html
     * @see http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/transforms/index.htm
     * @see http://content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation
     * @see http://webcache.googleusercontent.com/search?q=cache:xgJAp3bDNhQJ:content.gpwiki.org/index.php/OpenGL:Tutorials:Using_Quaternions_to_represent_rotation&hl=en&gl=us&strip=1
     * 
     * @see VectorInt16::getProduct(Quaternion)
     */
    void rotate(Quaternion *q) {

        /**
         * @details Uses the formula:
         * 
         *  - `P_out = q * P_in * conj(q)`:
         *      - `P_out` is the output vector
         *      - `q` is the orientation quaternion
         *      - `P_in` is the input vector (a*aReal)
         *      - `conj(q)` is the conjugate of the quaternion:
         *          - if q := [w, x, y, z], q* = [w, -x, -y, -z]
         * 
         *  - broken up into steps:
         *      1. treat vector as quaternion with w=0
         *      2. p is assigned `q * p`
         *      3. p is then assigned `p * conj(q)`
         *          1. substituting p from step 2: `p := (q * p) * conj(q)`
         *      4. p quaternion is now [0, x', y', z']
         */

        // treats vector as quaternion with w=0
        Quaternion p(0, x, y, z);

        // quaternion multiplication: q * p, stored back in p
        p = q->getProduct(p);

        // quaternion multiplication: p * conj(q), stored back in p
        p = p.getProduct(q->getConjugate());

        // vector is assigned quaternion values `[0, x', y', z']`
        x = p.x;
        y = p.y;
        z = p.z;
    }

    /**
     * @brief Get the Rotated object. Unlike `rotate()`, this returns a rotated
     *  copy of this vector.
     * 
     * @param[in] q Quaternion to rotate by
     * @return VectorInt16 
     * 
     * @see rotate(Quaternion*)
     */
    VectorInt16 getRotated(Quaternion*  q) {
        VectorInt16 r(x, y, z);
        r.rotate(q);
        return r;
    }

    VectorInt16 operator+(const VectorInt16 &a) {
        return VectorInt16( x+a.x, y+a.y, z+a.z ); 
    }

    VectorInt16 operator-(const VectorInt16 &a) {
        return VectorInt16( x-a.x, y-a.y, z-a.z ); 
    }

    VectorInt16 operator*(const VectorInt16 &a) {
        return VectorInt16( x*a.x, y*a.y, z*a.z ); 
    }

    VectorInt16 operator*(const float &a) {
        return VectorInt16( a*x, a*y, a*z ); 
    }

    VectorInt16 operator/(const VectorInt16 &a) {
        return VectorInt16( x/a.x, y/a.y, z/a.z ); 
    }

    VectorInt16 operator/(const float &a) {
        return VectorInt16( x/a, y/a, z/a ); 
    }
    
    VectorInt16 print() {
        Serial.print(x); Serial.print("\t");
        Serial.print(y); Serial.print("\t");
        Serial.print(z);
        return *this;
    }
};

class VectorFloat {
public:
    float x;
    float y;
    float z;

    VectorFloat() {
        x = 0;
        y = 0;
        z = 0;
    }
    
    VectorFloat(float nx, float ny, float nz) {
        x = nx;
        y = ny;
        z = nz;
    }
    VectorFloat(const VectorInt16& a) { 
        x = (float)a.x;  
        y = (float)a.y;  
        z = (float)a.z; 
    }

    inline VectorFloat getProduct(const VectorFloat* v) {
        // VectorFloat multiplication is defined by:
        //     (V1 * V2).x = (y1z2 - z1y2)
        //     (V1 * V2).y = (z1x2 - x1z2)
        //     (V1 * V2).z = (x1y2 - y1x2)
        
        return VectorFloat(
            y*v->z - z*v->y,  // new x
            z*v->x - x*v->z,  // new y
            x*v->y - y*v->x   // new z
        );
    }

    float getMagnitude() {
        return sqrt(x*x + y*y + z*z);
    }

    void normalize() {
        float m = getMagnitude();
        // if (m < 0.0001) return;
        x /= m;
        y /= m;
        z /= m;
    }
    
    VectorFloat getNormalized() {
        VectorFloat r(x, y, z);
        r.normalize();
        return r;
    }
    
    VectorFloat operator+(const VectorFloat &a) {
        return VectorFloat( x+a.x, y+a.y, z+a.z );
    }

    VectorFloat operator-(const VectorFloat &a) {
        return VectorFloat( x-a.x, y-a.y, z-a.z );
    }

    VectorFloat operator*(const VectorFloat &a) {
        return VectorFloat( x*a.x, y*a.y, z*a.z );
    }

    VectorFloat operator*(const float &a) {
        return VectorFloat( a*x, a*y, a*z );
    }

    VectorFloat operator/(const VectorFloat &a) {
        return VectorFloat( x/a.x, y/a.y, z/a.z );
    }

    VectorFloat operator/(const float &a) {
        return VectorFloat( x/a, y/a, z/a ); 
    }
    
    void rotate(Quaternion *q) {
        Quaternion p(0, x, y, z);

        // quaternion multiplication: q * p, stored back in p
        p = q->getProduct(p);

        // quaternion multiplication: p * conj(q), stored back in p
        p = p.getProduct(q->getConjugate());

        // p quaternion is now [0, x', y', z']
        x = p.x;
        y = p.y;
        z = p.z;
    }

    VectorFloat getRotated(Quaternion *q) {
        VectorFloat r(x, y, z);
        r.rotate(q);
        return r;
    }
    
    VectorFloat print(uint8_t sigfig = 3) {
        
        Serial.print(x, sigfig); Serial.print("\t");
        Serial.print(y, sigfig); Serial.print("\t");
        Serial.print(z, sigfig);
        return *this;
    }
};


// ----------------------------------------------------------------------------
//
//  DATA STRUCTURES & CONSTANTS
//
// ----------------------------------------------------------------------------

/**
 * @brief Simple vector structure, in fixed point integers
 * 
 * @param x X-axis component
 * @param y Y-axis component
 * @param z Z-axis component
 */
typedef struct {

    int16_t x, y, z;

} vector_int_t;


/**
 * @brief Simple vector structure, in flaots
 * 
 * @param x X-axis component
 * @param y Y-axis component
 * @param z Z-axis component
 */
typedef struct {

    float x, y, z;

} vector_float_t;


/**
 * @brief ICM-20948 data container
 * 
 * @param accel  Accelerometer data
 * @param gyro   Gyroscope data
 * @param temp   Temperature data
 * @param counts Sample count or timestamp
 */
typedef struct {

    vector_int_t    accel;
    vector_int_t    gyro;
    int16_t         temp;
    uint32_t        counts;

} icm_data_t;


/**
 * @brief AK09916 magnetometer data container
 * 
 * @param mag    Magnetometer data
 * @param counts Sample count or timestamp
 */
typedef struct {

    vector_int_t    mag;
    uint32_t        counts;

} ak_data_t;


/**
 * @brief Generic data frame container
 * 
 * @param accel  Accelerometer data
 */
typedef struct {

    // vector_int_t    accel;
    // vector_int_t    gyro;
    // int16_t         temp;
    // vector_int_t    mag;

    // vector_float_t  accel;
    // vector_float_t  gyro;
    // int16_t         temp;
    // vector_float_t  mag;
    // Quaternion      quat;
    // uint32_t        counts;

    VectorFloat     accel;
    VectorFloat     gyro;
    int16_t         temp;
    VectorFloat     mag;
    Quaternion      q;
    uint32_t        counts;

} dataframe_t;

typedef struct {
  float time; // added

  VectorFloat accel,
              gyro,
              mag;

  // from FOV of world
  Quaternion q;

} States;



// ----------------------------------------------------------------------------
// 
//  HELPER FUNCTIONS
// 
// ----------------------------------------------------------------------------

/**
 * @brief returns linear acceleration vector (gravity removed)
 * 
 * @param[in] vRaw raw acceleration vector
 * @param[in] gravity gravity vector
 * @param[out] v linear acceleration vector (gravity removed)
 * @return uint8_t confirms successful execution
 * @retval 0 on success
 */
inline uint8_t Get_Linear_Accel(
        VectorInt16*    vRaw,
        VectorFloat*    gravity,
        VectorInt16*    v)
{
    // get rid of the gravity component (+1g = +8192 in standard DMP FIFO
    // packet, sensitivity is 2g)
    // v->x    = vRaw->x - gravity->x*8192;
    // v->y    = vRaw->y - gravity->y*8192;
    // v->z    = vRaw->z - gravity->z*8192;

    // packet, sensitivity is 2g)
    v->x    = vRaw->x - gravity->x*8192;
    v->y    = vRaw->y - gravity->y*8192;
    v->z    = vRaw->z - gravity->z*8192;

    return 0;
}

/**
 * @brief returns linear acceleration vector (gravity removed)
 * 
 * @param[in] vRaw raw acceleration vector
 * @param[in] gravity gravity vector
 * @param[out] v linear acceleration vector (gravity removed)
 * @return uint8_t confirms successful execution
 * @retval 0 on success
 */
inline uint8_t Get_Linear_Accel(
        VectorFloat*    vRaw,
        VectorFloat*    gravity,
        VectorFloat*    v)
{
    // get rid of the gravity component (+1g = +8192 in standard DMP FIFO
    // packet, sensitivity is 2g)
    // v->x    = vRaw->x - gravity->x*8192;
    // v->y    = vRaw->y - gravity->y*8192;
    // v->z    = vRaw->z - gravity->z*8192;

    // packet, sensitivity is 2g)
    v->x    = vRaw->x - gravity->x;
    v->y    = vRaw->y - gravity->y;
    v->z    = vRaw->z - gravity->z;

    return 0;
}


/**
 * @brief returns linear acceleration vector in world frame of reference
 * 
 * @param[in] vReal world-frame linear acceleration vector
 * @param[in] q orientation quaternion
 * @param[out] v linear acceleration vector in world frame of reference
 * @return uint8_t confirms successful execution
 * @retval 0 on success
 */
inline uint8_t Get_World_Accel(
    VectorInt16*    vReal,
    Quaternion*     q,
    VectorInt16*    v)
{
    // rotate measured 3D acceleration vector into original state
    // frame of reference based on orientation quaternion
    memcpy(v, vReal, sizeof(VectorInt16));
    v->rotate(q);

    return 0;
}


/**
 * @brief returns gravity vector based on orientation quaternion
 * 
 * @param[in] q orientation quaternion
 * @param[out] v linear acceleration vector in world frame of reference
 * @return uint8_t confirms successful execution
 * @retval 0 on success
 */
inline uint8_t Get_Gravity(
        Quaternion*     q,
        VectorFloat*    v
    )
{
    v->x    = 2*(q->x*q->z - q->w*q->y);
    v->y    = 2*(q->w*q->x + q->y*q->z);
    v->z    = q->w*q->w - q->x*q->x - q->y*q->y + q->z*q->z;
    return 0;
}

/**
 * @brief returns Euler angles from orientation quaternion
 * 
 * @param[in] q orientation quaternion
 * @param[out] data array of size 3 to hold Euler angles (psi, theta, phi)
 * @return uint8_t confirms successful execution
 * @retval 0 on success
 */
inline uint8_t Get_Euler(
        Quaternion* q,
        float*      data)
{
    data[0] =  atan2(2*q->x*q->y - 2*q->w*q->z,
                     2*q->w*q->w + 2*q->x*q->x - 1);    // psi
    data[1] = -asin( 2*q->x*q->z + 2*q->w*q->y);        // theta
    data[2] =  atan2(2*q->y*q->z - 2*q->w*q->x,
                     2*q->w*q->w + 2*q->z*q->z - 1);    // phi

    return 0;
}

// TODO: implement the functions 
// inline is used to avoid "multiple definition" errors during linking
// if I am done making the changes to this file, I can add the function definitions
// to a .cpp file and remove the inline keywords




#endif /* _HELPER_3DMATH_H_ */
