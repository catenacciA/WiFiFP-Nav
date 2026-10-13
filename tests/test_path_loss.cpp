#include "wififp/path_loss.hpp"

#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>

using namespace wififp;

TEST(PathLoss, ReferenceDistanceGivesP0) {
  // With the softening, the effective distance at the AP itself is d0, where the model equals P0.
  const gtsam::Vector2 params(-40.0, 2.0);
  const double at_ap = predictRssi(gtsam::Point3(0, 0, 0), gtsam::Point3(0, 0, 0), params);
  EXPECT_NEAR(at_ap, -40.0, 1e-12);
}

TEST(PathLoss, DecaysWithDistance) {
  const gtsam::Vector2 params(-40.0, 3.0);
  const gtsam::Point3 ap(0, 0, 0);
  const double near = predictRssi(gtsam::Point3(2, 0, 0), ap, params);
  const double far = predictRssi(gtsam::Point3(20, 0, 0), ap, params);
  EXPECT_GT(near, far);
  // Ten times farther costs ~10 n dB once well beyond d0.
  EXPECT_NEAR(predictRssi(gtsam::Point3(100, 0, 0), ap, params) - predictRssi(gtsam::Point3(1000, 0, 0), ap, params),
              30.0, 0.01);
}

TEST(PathLoss, AnalyticJacobiansMatchNumerical) {
  const gtsam::Point3 rx(1.3, -2.1, 0.8), ap(-0.7, 3.2, 2.4);
  const gtsam::Vector2 params(-37.5, 2.7);

  gtsam::Matrix13 H_rx, H_ap;
  gtsam::Matrix12 H_params;
  predictRssi(rx, ap, params, H_rx, H_ap, H_params);

  const std::function<double(const gtsam::Point3&, const gtsam::Point3&, const gtsam::Vector2&)> f =
      [](const gtsam::Point3& a, const gtsam::Point3& b, const gtsam::Vector2& p) { return predictRssi(a, b, p); };
  using gtsam::Matrix;
  EXPECT_TRUE(gtsam::assert_equal(Matrix(gtsam::numericalDerivative31(f, rx, ap, params)), Matrix(H_rx), 1e-7));
  EXPECT_TRUE(gtsam::assert_equal(Matrix(gtsam::numericalDerivative32(f, rx, ap, params)), Matrix(H_ap), 1e-7));
  EXPECT_TRUE(gtsam::assert_equal(Matrix(gtsam::numericalDerivative33(f, rx, ap, params)), Matrix(H_params), 1e-7));
}
