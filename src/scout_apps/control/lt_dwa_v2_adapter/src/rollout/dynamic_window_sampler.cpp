#include "lt_dwa_v2_adapter/rollout/dynamic_window_sampler.h"

#include <algorithm>
#include <cmath>

namespace lt_dwa_v2_adapter
{
namespace
{
double clamp(double value, double lo, double hi)
{
  return std::max(lo, std::min(hi, value));
}

std::vector<double> evenlySpaced(double lo, double hi, int samples, double fallback)
{
  std::vector<double> values;
  if (samples <= 1 || std::abs(hi - lo) < 1e-9)
  {
    values.push_back(clamp(fallback, lo, hi));
    return values;
  }
  values.reserve(static_cast<size_t>(samples));
  for (int i = 0; i < samples; ++i)
  {
    const double r = static_cast<double>(i) / static_cast<double>(samples - 1);
    values.push_back(lo + r * (hi - lo));
  }
  return values;
}
}  // namespace

DynamicWindowSampler::DynamicWindowSampler(const PlannerLimits& limits,
                                           const SamplingConfig& sampling,
                                           const RolloutConfig& rollout)
{
  configure(limits, sampling, rollout);
}

void DynamicWindowSampler::configure(const PlannerLimits& limits,
                                     const SamplingConfig& sampling,
                                     const RolloutConfig& rollout)
{
  limits_ = limits;
  sampling_ = sampling;
  rollout_ = rollout;

  limits_.v_max_mps = std::max(0.0, limits_.v_max_mps);
  limits_.omega_max_radps = std::max(0.0, limits_.omega_max_radps);
  limits_.a_max_mps2 = std::max(0.0, limits_.a_max_mps2);
  limits_.alpha_max_radps2 = std::max(0.0, limits_.alpha_max_radps2);
  sampling_.v_samples = std::max(1, sampling_.v_samples);
  sampling_.omega_samples = std::max(1, sampling_.omega_samples);
  rollout_.dt = std::max(0.02, rollout_.dt);
}

DynamicWindow DynamicWindowSampler::windowFor(const RobotState& state) const
{
  const double linear_step = limits_.a_max_mps2 * rollout_.dt;
  const double angular_step = limits_.alpha_max_radps2 * rollout_.dt;
  const double global_min_v = limits_.allow_reverse ? -limits_.v_max_mps : 0.0;

  DynamicWindow window;
  window.min_v = clamp(state.v - linear_step, global_min_v, limits_.v_max_mps);
  window.max_v = clamp(state.v + linear_step, global_min_v, limits_.v_max_mps);
  window.min_omega = clamp(state.omega - angular_step, -limits_.omega_max_radps, limits_.omega_max_radps);
  window.max_omega = clamp(state.omega + angular_step, -limits_.omega_max_radps, limits_.omega_max_radps);
  return window;
}

std::vector<double> DynamicWindowSampler::sampleLinearVelocities(double current_v) const
{
  RobotState state;
  state.v = current_v;
  const DynamicWindow window = windowFor(state);
  return evenlySpaced(window.min_v, window.max_v, sampling_.v_samples, current_v);
}

std::vector<double> DynamicWindowSampler::sampleAngularVelocities(double current_omega) const
{
  RobotState state;
  state.omega = current_omega;
  const DynamicWindow window = windowFor(state);
  return evenlySpaced(window.min_omega, window.max_omega, sampling_.omega_samples, current_omega);
}

std::vector<Command> DynamicWindowSampler::sampleCommands(const RobotState& state) const
{
  const auto v_samples = sampleLinearVelocities(state.v);
  const auto omega_samples = sampleAngularVelocities(state.omega);
  std::vector<Command> commands;
  commands.reserve(v_samples.size() * omega_samples.size());
  for (const double v : v_samples)
  {
    for (const double omega : omega_samples)
    {
      Command command;
      command.v = v;
      command.omega = omega;
      commands.push_back(command);
    }
  }
  return commands;
}
}  // namespace lt_dwa_v2_adapter
