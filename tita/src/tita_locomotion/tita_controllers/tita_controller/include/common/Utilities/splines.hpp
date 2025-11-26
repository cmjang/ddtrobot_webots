#ifndef SPLINE_HPP__
#define SPLINE_HPP__
#include <cmath>
// usage 1.spline update 2.get value 3. dt++ 4.done and reset
/*
    QuinticSpline<double> spline;
    spline.setSpline(0.13575,0,0,0.32,0,40,0.1);
    while(1)
    {
        spline.update();
        spline._stamp += 0.005;
        if(spline._done)  break;
    }
    spline.reset();
*/
template <typename T>
class Spline
{
public:
  virtual void update(T dt) = 0;
  T getPos() { return this->_pos; }
  T getVel() { return this->_vel; }
  T getAccl() { return this->_accl; }
  void reset()
  {
    this->_done = false;
    this->_stamp = 0;
  }

public:
  bool _done;

protected:
  T _stamp = 0;
  T _tf;
  T _pos = 0.0f, _vel = 0.0f, _accl = 0.0f;
  T _EPS = 10e-5;
};

template <typename T>
class QuinticSpline : public Spline<T>
{
public:
  QuinticSpline()
  {
    this->_done = false;
    this->_stamp = 0;
  }
  void setQuintic(T x0, T v0, T a0, T xf, T vf, T af, T td)
  {
    // Calculate the coefficients of the quintic spline trajectory
    this->_stamp = 0;
    this->_tf = td;
    this->_done = false;
    T t1 = td;
    T t2 = pow(td, 2);
    T t3 = pow(td, 3);
    T t4 = pow(td, 4);
    T t5 = pow(td, 5);
    this->_a = x0;
    this->_b = v0;
    this->_c = 0.5 * a0;
    this->_d = (20 * (xf - x0) - (8 * vf + 12 * v0) * t1 - (3 * a0 - af) * t2) / (2 * t3);
    this->_e = (30 * (x0 - xf) + (14 * vf + 16 * v0) * t1 + (3 * a0 - 2 * af) * t2) / (2 * t4);
    this->_f = (12 * (xf - x0) - 6 * (vf + v0) * t1 - (a0 - af) * t2) / (2 * t5);
  }

  void update(T dt)
  {
    T t = this->_stamp;
    // Output the current position, velocity, and acceleration at time t
    if (!this->_done) {
      this->_pos = _a + _b * t + _c * pow(t, 2) + _d * pow(t, 3) + _e * pow(t, 4) + _f * pow(t, 5);
      this->_vel = _b + 2 * _c * t + 3 * _d * pow(t, 2) + 4 * _e * pow(t, 3) + 5 * _f * pow(t, 4);
      this->_accl = 2 * _c + 6 * _d * t + 12 * _e * pow(t, 2) + 20 * _f * pow(t, 3);
    }
    if (fabs(this->_tf - t <= this->_EPS)) {
      this->_done = true;
    } else {
      this->_done = false;
    }
    this->_stamp += dt;
  }

  bool isFinish() { return this->_done; }
  // public:
  //     bool _done;

private:
  T _a, _b, _c, _d, _e, _f;
};

// template<typename T>
// class Ramp2 : public Spline<T> {
// public:
//     Ramp2()
//     {
//         this->_done = false;
//         this->_stamp = 0;
//     };
//     void setRamp(T x0, T xf, T td) {
//         // Calculate the coefficients of the ramp trajectory
//         this->_stamp = 0;
//         this->_tf = td;
//         this->_done = false;
//         this->_a = x0;
//         this->_b = (xf - x0) / td;
//     }

//     void update(T dt) {
//         this->_stamp += dt;
//         T t = this->_stamp;
//         // Output the current position, velocity, and acceleration at time t
//         this->_pos = _a + _b * t;
//         this->_vel = _b;
//         this->_accl = 0;
//         if(fabs(this->_tf - t<=_EPS))   { this->_done = true; }
//         else                              this->_done = false;
//     }
// public:
//     bool _done;
// private:
//     T _stamp = 0;
//     T _a, _b, _c, _d;
//     T _EPS = 10e-5;
// };
#endif