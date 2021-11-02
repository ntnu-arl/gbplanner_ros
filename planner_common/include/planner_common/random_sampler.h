#ifndef RANDOM_SAMPLER_H_
#define RANDOM_SAMPLER_H_

#include <memory>
#include <random>

#include "planner_common/params.h"

class RandomSamplerBase {
 public:
  enum RandomDistributionType {
    kConst,    // Perhaps you want to a set constant value.
    kUniform,  // Uniformly sample, have to define a bound.
    kNormal,   // Normally sample, have to define the mean and std.
    kCauchy    // Samply with Cauchy (heavier tail than kNormal)
  };

  enum SampleModeType {
    kManual,  // Parse params manually from the setting in yaml file.
    kLocal,  // Get from locally bounded space automatically. (Only in kUniform)
    kGlobal,       // Get from globally bounded space automatically.(Only in
                   // kUniform)
    kExploredMap,  // Get bound online from current explored map boundaries.
                   // (Only in kUniform)
    kIgnore        // No sampling, return a const value 0.0.
  };

  RandomSamplerBase();
  ~RandomSamplerBase();
  bool loadParams(std::string ns);
  void setParams(int ind, BoundedSpaceParams& global_params,
                 BoundedSpaceParams& local_params);
  void setBound(double min_val, double max_val);
  void setDistributionParams(double mean_val, double std_val);
  void reset();
  double generate(double current_val, double devisor = 1);
  double getZOffset();

 private:
  std::mt19937 generator_;
  RandomDistributionType pdf_type_;
  SampleModeType sample_mode_;
  double const_val_;
  double mean_val_;
  double std_val_;
  double min_val_;
  double max_val_;
  bool use_bound_;  // In case of pdf others than uniform, this is true when
                    // want to truncate the sample value to min_val and max_val

  std::shared_ptr<std::uniform_real_distribution<double>> uniform_pdf_;
  std::shared_ptr<std::normal_distribution<double>> normal_pdf_;
  std::shared_ptr<std::cauchy_distribution<double>> cauchy_pdf_;
};

class RandomSampler {
 public:
  RandomSampler();
  ~RandomSampler();
  bool loadParams(std::string ns);
  bool setParams(BoundedSpaceParams& global_params,
                 BoundedSpaceParams& local_params, bool rotation = true);
  // To set the bound online.
  bool setBound(Eigen::Vector3d& min_val, Eigen::Vector3d& max_val);
  bool setRotation(Eigen::Vector3d& rotations);
  bool setDistributionParams(Eigen::Vector3d& mean_val,
                             Eigen::Vector3d& std_val);

  void reset();
  void generate(StateVec& current_state, StateVec& sample_state);

  void pushSample(StateVec& state, bool valid);
  std::vector<StateVec>* getSamples(bool valid) {
    if (valid)
      return &valid_samples_;
    else
      return &invalid_samples_;
  }
  // A hack function to get the offset setting in Z-axis for ANYmal.
  double getZOffset() {
    const int kZIndex = 2;
    if (random_sampler_base_.size() >= 3)
      return random_sampler_base_[kZIndex].getZOffset();
    else
      return 0.0;
  }

  bool isReady = false;

 private:
  std::vector<RandomSamplerBase> random_sampler_base_;
  const int kValidSamplesMaxSize = 1000;
  const int kInvalidSamplesMaxSize = 1000;
  std::vector<StateVec> valid_samples_;
  std::vector<StateVec> invalid_samples_;
  Eigen::Matrix3d rot_B2W;

  // Used to generate multivariate Cauchy distribution
  std::mt19937 generator_;
  std::shared_ptr<std::chi_squared_distribution<double>> chi_squared_;
};

#endif
