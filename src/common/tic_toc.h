#ifndef POINT_CLOUD_REGISTRATION_TICTOC_H_
#define POINT_CLOUD_REGISTRATION_TICTOC_H_

#include <chrono>
#include <cstdlib>
#include <ctime>

namespace pcr {
class TicToc {
   public:
    TicToc() { Tic(); }

    void Tic() { start_ = std::chrono::system_clock::now(); }

    double Toc() {
        end_ = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_ - start_;
        return elapsed_seconds.count() * 1000;
    }

   private:
    std::chrono::time_point<std::chrono::system_clock> start_, end_;
};
}  // namespace pcr

#endif
