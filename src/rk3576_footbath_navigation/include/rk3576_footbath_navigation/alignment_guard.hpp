#pragma once
#include <cmath>
#include <stdexcept>
namespace footbath {
// Unwrap one alignment attempt across +/- pi without alternating direction.
class AlignmentGuard {
public:
  void reset() { started_ = false; }
  double update(double now, double wrapped_error) {
    if (!std::isfinite(now) || !std::isfinite(wrapped_error))
      throw std::runtime_error("Invalid alignment input");
    if (!started_) {
      started_ = true; start_ = progress_at_ = now;
      error_ = previous_ = wrapped_error; best_ = std::abs(error_);
    } else {
      error_ += std::remainder(wrapped_error - previous_, 2.0 * M_PI);
      previous_ = wrapped_error;
    }
    if (std::abs(error_) < best_ - 0.03) {
      best_ = std::abs(error_); progress_at_ = now;
    }
    if (now < start_ || now - start_ > 20.0 || now - progress_at_ > 3.0)
      throw std::runtime_error("Initial alignment stopped converging or timed out");
    return error_;
  }
private:
  bool started_{false};
  double start_{0.}, progress_at_{0.}, error_{0.}, previous_{0.}, best_{0.};
};
}
