//
// Created by dcheng on 1/20/21.
//

#ifndef SRC_SENSITIVITY_ANALYSER_H
#define SRC_SENSITIVITY_ANALYSER_H

#include <camodocal/camera_models/CameraFactory.h>
#include <camodocal/camera_models/CataCamera.h>

class SensitivityAnalyser
{
public:
  SensitivityAnalyser(const std::string& config_fn);

  void evalSensitivity();

  void evalSensitivityAtDiameters(std::vector<double> diameters);

  void evalSensitivityAtBaselines(std::vector<double> baselines);

  double evalSensitivityAtDiameterBaseline(double diameter, double baseline);

private:
  void evalSensitivityAtDiameter(double diameter,
                                 std::vector<double> &baselines,
                                 std::vector<double> &sensitivities);

  void evalSensitivityAtBaseline(double baseline,
                                 std::vector<double> &diameters,
                                 std::vector<double> &sensitivities);

  double sensitivity2precision(double sensitivity);
  void sensitivity2precision(const std::vector<double> &sensitivities,
                             std::vector<double> &precisions);

  camodocal::CameraPtr m_camera_;

  constexpr static const double MAX_BASELINE = 0.3;
  constexpr static const double MIN_BASELINE = 0.02;
  constexpr static const double MAX_DIAMETER = 0.6;
  constexpr static const double MIN_DIAMETER = 0.1;
};

#endif //SRC_SENSITIVITY_ANALYSER_H

