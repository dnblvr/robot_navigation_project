
#ifndef _ECF_H_
#define _ECF_H_

#include "helper_3dmath.h"
// #include "data_structure.h"
// #include "fixed_point.h"


// TODO: make a configuration class thing

// TODO: set up fixed point preprocessing
#ifdef _FIXED_POINT_H_ // defined when fixed_point.h is live

#else

  class ExtendedComplementaryFilter {

    public: 

      ExtendedComplementaryFilter();

      // ExtendedComplementaryFilter(); // TODO: integrate this with the configuration class

      ExtendedComplementaryFilter(
            float K_norm, 
            float t_norm, 
            float K_init, 
            float dt) :
            K_norm_(K_norm), 
            t_norm_(t_norm), 
            K_init_(K_init), 
            dt_(dt), 
            t_ratio_(t_norm/dt),
            ref_accel_(0, 0, -1),
            ref_mag_  (0, -1, 0)
      {} // TODO: integrate this with the configuration class


      void update(
            dataframe_t* np1,
            dataframe_t* n)
        {

        Quaternion q_p      = n->q;

        float accel_norm_n_ = np1->accel.getMagnitude(),
              mag_norm_n_   = np1->mag.getMagnitude();

        VectorFloat unit_accel_n  = np1->accel.getNormalized();
        VectorFloat unit_mag_n    = np1->mag.getNormalized();

        Quaternion  av_n_;
        VectorFloat gyro_ddt_;

        // temporary
        VectorFloat mag_east = unit_accel_n.getProduct(&unit_mag_n);

        float time = np1->counts*dt_;

        
        // to set the conditions for the calculations, see (26) ---------------
        total_error_n_ = VectorFloat(); // initialized to zero


        // add the accel error term
        if (accel_norm_n_ > 0) {

          // this rotates the reference gravity unit vectors -----------
          local_accel_n_ = VectorFloat(
                2*(q_p.x*q_p.z - q_p.w*q_p.y),
                2*(q_p.w*q_p.x + q_p.y*q_p.z),
                2*(q_p.w*q_p.w + q_p.z*q_p.z) - 1
          );

          total_error_n_ = total_error_n_ + unit_accel_n.getProduct(&local_accel_n_);  // see (12)
        }

        // add the magnetic error term
        if ( MIN_MAG_ < mag_norm_n_ && mag_norm_n_ < MAX_MAG_ ) {

          // this rotates the reference gravity and geomagnetic unit vectors -----------
          local_mag_n_   = VectorFloat( 
                2*(q_p.x*q_p.y + q_p.w*q_p.z),
                2*(q_p.w*q_p.w + q_p.z*q_p.z) - 1,
                2*(q_p.y*q_p.z - q_p.w*q_p.x)
          );

          total_error_n_ = total_error_n_ + mag_east.getProduct(&local_mag_n_);

        } else {

          // instead, this rotates the reference mag
          local_mag_n_ = ref_mag_.getRotated(&q_p);

          total_error_n_ = total_error_n_ + mag_east.getProduct(&local_mag_n_);
        }

        // calculate angular velocity using gyroscopic data
        K_n_      = find_gain_(time);

        gyro_ddt_ = np1->gyro - total_error_n_*K_n_;
        av_n_     = Quaternion(0, gyro_ddt_.x, gyro_ddt_.y, gyro_ddt_.z);
        q_ddt_    = q_p.getProduct(av_n_) * 0.5f;

        np1->q = q_p + q_ddt_*dt_;
        np1->q.normalize();

      }

    private:
      
      // input variables
      float K_norm_, t_norm_, K_init_, K_n_,

            dt_; // microseconds;
      
      float t_ratio_;

      unsigned int num_samples;

      // min and max mag values
      const float MIN_MAG_  = 2000, // microTesla
                  MAX_MAG_  = 6500; // microTesla

      // containers for storing the result of rotated accel/mag vectors
      VectorFloat ref_accel_, 
                  ref_mag_,

                  local_accel_n_,
                  local_mag_n_,
                  
                  total_error_n_;
      

      // global reference frame relative to the IMU
      Quaternion  gyro_ddt_,
                  q_ddt_;
      
      inline float find_gain_(float time) {
        return K_norm_   
                +   (time < t_norm_) * (t_norm_ - time)*(K_init_ - K_norm_)/t_norm_;
      }
      
      inline float find_gain_(unsigned int sample) {
        return K_norm_   +   ((float)sample < t_norm_) * (t_ratio_ - (float)sample)*(K_init_ - K_norm_)/t_ratio_;
      }

  };

  #endif // _FIXED_POINT_H_

#endif // _ECF_H_