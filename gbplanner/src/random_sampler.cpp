#include "gbplanner/random_sampler.h"

namespace explorer {

RandomSamplerBase::RandomSamplerBase() {
  mean_val_ = 0.0;
  std_val_ = 0.0;
  min_val_ = 0.0;
  max_val_ = 0.0;
}

RandomSamplerBase::~RandomSamplerBase() {}

bool RandomSamplerBase::loadParams(std::string ns) {
  ROSPARAM_INFO("Loading: " + ns);
  std::string param_name;
  std::vector<double> param_val;
  std::string parse_str;

  param_name = ns + "/pdf_type";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kUniform"))
    pdf_type_ = RandomDistributionType::kUniform;
  else if (!parse_str.compare("kNormal"))
    pdf_type_ = RandomDistributionType::kNormal;
  else if (!parse_str.compare("kConst"))
    pdf_type_ = RandomDistributionType::kConst;
  else {
    ROSPARAM_ERROR(ns);
    return false;
  }

  param_name = ns + "/sample_mode";
  ros::param::get(param_name, parse_str);
  if (!parse_str.compare("kManual"))
    sample_mode_ = SampleModeType::kManual;
  else if (!parse_str.compare("kLocal")) {
    sample_mode_ = SampleModeType::kLocal;
    // Have to load later from the BoundedSpaceParams/Local
  } else if (!parse_str.compare("kGlobal")) {
    sample_mode_ = SampleModeType::kGlobal;
    // Have to load later from the BoundedSpaceParams/Global
  } else if (!parse_str.compare("kExploredMap")) {
    sample_mode_ = SampleModeType::kExploredMap;
    // Have to be set online.
  } else {
    ROSPARAM_ERROR(ns);
    return false;
  }

  if (sample_mode_ == SampleModeType::kManual) {
    if (pdf_type_ == RandomDistributionType::kUniform) {
      // Uniform pdf: min, max
      param_name = ns + "/min_val";
      if (!ros::param::get(param_name, min_val_)) {
        ROSPARAM_ERROR(param_name);
        return false;
      }
      param_name = ns + "/max_val";
      if (!ros::param::get(param_name, max_val_)) {
        ROSPARAM_ERROR(param_name);
        return false;
      }
    } else if (pdf_type_ == RandomDistributionType::kNormal) {
      // Normal pdf: mean, std, as well as min and max to truncate if required
      use_bound_ = true;
      param_name = ns + "/min_val";
      if (!ros::param::get(param_name, min_val_)) {
        ROSPARAM_WARN(param_name, "not used");
        use_bound_ = false;
      }
      param_name = ns + "/max_val";
      if (!ros::param::get(param_name, max_val_)) {
        ROSPARAM_WARN(param_name, "not used");
        use_bound_ = false;
      }
      param_name = ns + "/mean_val";
      if (!ros::param::get(param_name, mean_val_)) {
        ROSPARAM_ERROR(param_name);
        return false;
      }
      param_name = ns + "/std_val";
      if (!ros::param::get(param_name, std_val_)) {
        ROSPARAM_ERROR(param_name);
        return false;
      }
    } else if (pdf_type_ == RandomDistributionType::kConst) {
      // Set const value.
      param_name = ns + "/const_val";
      if (!ros::param::get(param_name, const_val_)) {
        ROSPARAM_ERROR(param_name);
        return false;
      }
    }
  }
  ROSPARAM_INFO("Done.");
  return true;
}

void RandomSamplerBase::setParams(int ind, BoundedSpaceParams& global_params,
                                  BoundedSpaceParams& local_params) {
  // First, load some parameters from bounded space settings.
  BoundedSpaceParams* params;
  if (SampleModeType::kGlobal == sample_mode_)
    params = &global_params;
  else if (sample_mode_ == SampleModeType::kLocal)
    params = &local_params;
  else
    return;

  if (pdf_type_ == RandomDistributionType::kUniform) {
    if (params->type == BoundedSpaceType::kCuboid) {
      min_val_ = params->min_val[ind];
      max_val_ = params->max_val[ind];
    } else if (params->type == BoundedSpaceType::kSphere) {
      min_val_ = -params->radius;
      max_val_ = params->radius;
    }
  }
}

void RandomSamplerBase::setBound(double min_val, double max_val) {
  // Set directly without checking mode.
  min_val_ = min_val;
  max_val_ = max_val;
}

void RandomSamplerBase::reset() {
  std::random_device rd;
  generator_.seed(rd());
  if (pdf_type_ == RandomDistributionType::kUniform) {
    uniform_pdf_.reset(
        new std::uniform_real_distribution<>(min_val_, max_val_));
  } else if (pdf_type_ == RandomDistributionType::kNormal) {
    normal_pdf_.reset(new std::normal_distribution<>(mean_val_, std_val_));
  }
}

double RandomSamplerBase::generate(double current_val) {
  double r;
  if (pdf_type_ == RandomDistributionType::kUniform) {
    r = (*uniform_pdf_)(generator_);
  } else if (pdf_type_ == RandomDistributionType::kNormal) {
    r = normal_pdf_->operator()(generator_);
  } else if (pdf_type_ == RandomDistributionType::kConst) {
    r = const_val_;
  } else {
    ROS_ERROR("Not supported yet.");
    return 0.0;
  }

  // Offset to current pose; except if this is global setting.
  if (sample_mode_ != SampleModeType::kGlobal) r += current_val;

  return r;
}

RandomSampler::RandomSampler() {
  random_sampler_base_.clear();
  random_sampler_base_.resize(4);
  valid_samples_.clear();
}
RandomSampler::~RandomSampler() {}

bool RandomSampler::loadParams(std::string ns) {
  if (!random_sampler_base_[0].loadParams(ns + "/X")) return false;
  if (!random_sampler_base_[1].loadParams(ns + "/Y")) return false;
  if (!random_sampler_base_[2].loadParams(ns + "/Z")) return false;
  if (!random_sampler_base_[3].loadParams(ns + "/Heading")) return false;
  isReady = true;
  return true;
}

bool RandomSampler::setBound(Eigen::Vector3d& min_val,
                             Eigen::Vector3d& max_val) {
  // This applied to x,y,z only
  for (int i = 0; i < 3; ++i) {
    random_sampler_base_[i].setBound(min_val[i], max_val[i]);
  }
  reset();
  return true;
}

bool RandomSampler::setParams(BoundedSpaceParams& global_params,
                              BoundedSpaceParams& local_params) {
  // This applied to x,y,z only
  for (int i = 0; i < 3; ++i) {
    random_sampler_base_[i].setParams(i, global_params, local_params);
  }
  reset();
  return true;
}

void RandomSampler::RandomSampler::reset() {
  for (int i = 0; i < 4; ++i) {
    random_sampler_base_[i].reset();
  }
  valid_samples_.clear();
  invalid_samples_.clear();
}

void RandomSampler::generate(StateVec& current_state, StateVec& sample_state) {
  for (int i = 0; i < 4; ++i) {
    sample_state[i] = random_sampler_base_[i].generate(current_state[i]);
  }
}

void RandomSampler::pushSample(StateVec& state, bool valid) {
  if (valid) {
    if (valid_samples_.size() < kValidSamplesMaxSize)
      valid_samples_.emplace_back(state);
  } else {
    if (invalid_samples_.size() < kInvalidSamplesMaxSize)
      invalid_samples_.emplace_back(state);
  }
}

}  // namespace explorer
