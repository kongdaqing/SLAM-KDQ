#include <iostream>
#include <chrono>

class Timer
{
private:
  std::chrono::steady_clock::time_point timeStartPoint_;
public:
  Timer(/* args */){tic();};
  ~Timer(){};
  inline void tic() {
    timeStartPoint_ = std::chrono::steady_clock::now();
  }
  inline double toc_ms() {
    std::chrono::steady_clock::time_point timeEndPoint =  std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsedTime = timeEndPoint - timeStartPoint_;
    timeStartPoint_ = timeEndPoint;
    return elapsedTime.count() * 1000;
  } 
};


