//
// Created by dcheng on 1/20/21.
//

#include <pipe_blaser_ros/sensitivity_analyser.h>
#include <third_party/matplotlibcpp.h>
#include <pipe_blaser_ros/common.h>

namespace plt = matplotlibcpp;

SensitivityAnalyser::SensitivityAnalyser(const std::string &config_fn)
{
  m_camera_ = camodocal::CameraFactory::instance()
      ->generateCameraFromYamlFile(config_fn);
}

void SensitivityAnalyser::evalSensitivity()
{
  static const int NUM_DATA = 100;
  std::vector<std::vector<double>> d, b, s; // diameter, baseline, sensitivity
  std::vector<std::vector<double>> p; // precision = 2 * pixel_error / sensitivity

  for (int i = 0; i < NUM_DATA; i++)
  {
    std::vector<double> d_row, b_row, s_row, p_row;
    double diameter = lerp(MIN_DIAMETER, MAX_DIAMETER, double(i) / NUM_DATA);

    int counter = 0;

    for (int j = 0; j < NUM_DATA; j++)
    {
      double baseline = lerp(MIN_BASELINE, MAX_BASELINE, double(j) / NUM_DATA);
      double sensitivity = evalSensitivityAtDiameterBaseline(diameter,
                                                             baseline);
      double precision = sensitivity2precision(sensitivity);

      // upper limit the precision
      if (precision > 2.5)
      {
        if (p_row.empty())
        {
          counter++;
        }
        else
        {
          d_row.push_back(d_row.back());
          b_row.push_back(b_row.back());
          s_row.push_back(s_row.back());
          p_row.push_back(p_row.back());
        }
        continue;
      }

      for (int add_i = 0; add_i <= counter; add_i ++)
      {
        d_row.push_back(diameter);
        b_row.push_back(baseline);
        s_row.push_back(sensitivity);
        p_row.push_back(precision);
      }
      counter = 0;
    }
    d.push_back(d_row);
    b.push_back(b_row);
    s.push_back(s_row);
    p.push_back(p_row);
  }

  /*
  plt::plot_surface(d, b, s);
  plt::xlabel("Diameter (m)");
  plt::ylabel("Baseline (m)");
  plt::set_zlabel("Sensitivity (pixel/mm)");
  plt::title("Sensitivity vs diameter & baseline");
  plt::show();
  */

  plt::plot_surface(d, b, p);
  plt::xlabel("Diameter (m)");
  plt::ylabel("Baseline (m)");
  plt::set_zlabel("Precision (mm)");
  plt::title("Precision vs diameter & baseline");
  plt::show();
}

void SensitivityAnalyser::evalSensitivityAtDiameter(double diameter,
                                                    std::vector<double> &baselines,
                                                    std::vector<double> &sensitivities)
{
  static const int NUM_DATA = 100;

  baselines.clear();
  sensitivities.clear();

  double max_sensitivity = -1., max_baseline;
  for (int i = 0; i < NUM_DATA; i++)
  {
    double baseline = lerp(MIN_BASELINE, MAX_BASELINE, double(i) / NUM_DATA);
    double sensitivity = evalSensitivityAtDiameterBaseline(diameter, baseline);
    baselines.push_back(baseline);
    sensitivities.push_back(sensitivity);

    if (sensitivity > max_sensitivity)
    {
      max_sensitivity = sensitivity;
      max_baseline = baseline;
    }
  }

  std::stringstream output_stream;
  output_stream << "At diameter = "
                << std::setprecision(3) << diameter
                << "m (" << diameter / 2.54 * 100
                << " inch), the best baseline length is " << max_baseline
                << " m with sensitivity = " << max_sensitivity
                << " pixel/mm." << endl;
  //cout << output_stream.str();

  /*
  plt::plot(baselines, sensitivities);
  std::stringstream str_stream;
  str_stream << "Sensitivity vs baseline at diameter = "
             << std::setprecision(2) << diameter
             << "m (" << diameter / 2.54 * 100 << " inch)";
  plt::title(str_stream.str());
  plt::xlabel("Baseline (m)");
  plt::ylabel("Sensitivity (pixel/mm)");
  plt::show();
   */
}

void
SensitivityAnalyser::evalSensitivityAtDiameters(std::vector<double> diameters)
{
  for (const auto diameter : diameters)
  {
    std::vector<double> baselines, sensitivities, precisions;
    evalSensitivityAtDiameter(diameter, baselines, sensitivities);

    sensitivity2precision(sensitivities, precisions);

    std::stringstream str_stream;
    str_stream << std::setprecision(2) << "Diameter = " << diameter
               << "m"; //<< diameter / 2.54 * 100 << " inch)";
    cout << "size 1: " << sensitivities.size() << endl;
    // cout << "size 2: " << sensitivities[0].size() << endl;
    // cout << sensitivities[1] << endl;
    for (int i = 0; i < sensitivities.size(); i++)
    {
     sensitivities[i] = 1/sensitivities[i];

    }

    plt::named_plot(str_stream.str(), baselines, sensitivities);
    //plt::named_plot(str_stream.str(), baselines, precisions);

  }

  plt::title("Accuracy vs baseline with various diameters (old configeration)");
  plt::xlabel("Baseline (m)");
  plt::ylabel("Accuracy (mm/pixel)");
  plt::legend();
  plt::show();

  /*
  plt::title("Precision vs baseline with various diameters");
  plt::xlabel("Baseline (m)");
  plt::ylabel("Precision (mm)");
  plt::legend();
  plt::show();
   */
}


double
SensitivityAnalyser::evalSensitivityAtDiameterBaseline(double diameter,
                                                       double baseline)
{
  Vector3d pt_c(0, diameter / 2, baseline);
  Vector3d pt_d_c(0, diameter / 2 + 1e-3, baseline);
  Vector2d uv, uv_d;
  m_camera_->spaceToPlane(pt_c, uv);
  m_camera_->spaceToPlane(pt_d_c, uv_d);

  return (uv - uv_d).norm();
}

double SensitivityAnalyser::sensitivity2precision(double sensitivity)
{
  return 2 * 0.5 / sensitivity;
}

void SensitivityAnalyser::sensitivity2precision(
    const std::vector<double> &sensitivities, std::vector<double> &precisions)
{
  precisions.clear();
  precisions.reserve(sensitivities.size());
  for (const double sensitivity : sensitivities)
    precisions.push_back(sensitivity2precision(sensitivity));
}

void
SensitivityAnalyser::evalSensitivityAtBaselines(std::vector<double> baselines)
{
  for (const auto baseline : baselines)
  {
    std::vector<double> diameters, sensitivities, precisions;
    evalSensitivityAtBaseline(baseline, diameters, sensitivities);

    sensitivity2precision(sensitivities, precisions);

    std::stringstream str_stream;
    str_stream << std::setprecision(2) << baseline * 100 << " cm";

    plt::named_plot(str_stream.str(), diameters, sensitivities);
    //plt::named_plot(str_stream.str(), diameters, precisions);

  }

  plt::title("Sensitivity vs diameter with various baselines");
  plt::xlabel("Diameter (m)");
  plt::ylabel("Sensitivity (pixel/mm)");
  plt::legend();
  plt::show();

  /*
  plt::title("Precision vs diameter with various baselines");
  plt::xlabel("Diameter (m)");
  plt::ylabel("Precision (mm)");
  plt::legend();
  plt::show();
   */
}

void SensitivityAnalyser::evalSensitivityAtBaseline(double baseline,
                                                    std::vector<double> &diameters,
                                                    std::vector<double> &sensitivities)
{
  static const int NUM_DATA = 100;

  diameters.clear();
  sensitivities.clear();

  double max_sensitivity = -1., max_diameter;
  for (int i = 0; i < NUM_DATA; i++)
  {
    double diameter = lerp(MIN_DIAMETER, MAX_DIAMETER, double(i) / NUM_DATA);
    double sensitivity = evalSensitivityAtDiameterBaseline(diameter, baseline);
    diameters.push_back(diameter);
    sensitivities.push_back(sensitivity);

    if (sensitivity > max_sensitivity)
    {
      max_sensitivity = sensitivity;
      max_diameter = diameter;
    }
  }

  std::stringstream output_stream;
  output_stream << "At baseline = "
                << std::setprecision(3) << baseline << " cm"
                << ", the best pipe diameter is " << max_diameter
                << " m with sensitivity = " << max_sensitivity
                << " pixel/mm." << endl;
  cout << output_stream.str();

  /*
  plt::plot(baselines, sensitivities);
  std::stringstream str_stream;
  str_stream << "Sensitivity vs baseline at diameter = "
             << std::setprecision(2) << diameter
             << "m (" << diameter / 2.54 * 100 << " inch)";
  plt::title(str_stream.str());
  plt::xlabel("Baseline (m)");
  plt::ylabel("Sensitivity (pixel/mm)");
  plt::show();
   */
}

int main(int argc, char **argv)
{
  SensitivityAnalyser analyser(argv[1]);
  //generates 3d plot of sensitivity vs (pipe_diameter, baseline)
  analyser.evalSensitivity();

  std::vector<double> pipe_diameters{0.1524, 0.2032, 0.3048, 0.4064};

  analyser.evalSensitivityAtDiameters(pipe_diameters);

  //std::vector<double> baselines{0.05, 0.1, 0.15, 0.2};
  std::vector<double> diameters{0.1524, 0.2333, 0.3167, 0.4};

  analyser.evalSensitivityAtBaselines(diameters);

  return 0;
}