#include "spmpc_local_planner/simulation/independent_scout_liquid_plant.h"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <vector>

namespace simulation = spmpc_local_planner::simulation;

namespace {

constexpr double kMotionCommandEndSec = 7.5;
constexpr double kCountEpsilon = 1.0e-12;

class ExclusiveOutputFile {
public:
    ~ExclusiveOutputFile() {
        if (fd_ >= 0) {
            ::close(fd_);
        }
        if (!preserved_ && !path_.empty()) {
            ::unlink(path_.c_str());
        }
    }

    bool reserve(const std::string& path, std::string& error) {
        const int fd = ::open(
            path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
        if (fd < 0) {
            error = errno == EEXIST
                ? "output already exists; refusing to overwrite: " + path
                : "cannot reserve output " + path + ": " +
                    std::strerror(errno);
            return false;
        }
        fd_ = fd;
        path_ = path;
        return true;
    }

    bool writeContents(const std::string& contents, std::string& error) {
        std::size_t offset = 0;
        while (offset < contents.size()) {
            const ssize_t written = ::write(
                fd_, contents.data() + offset, contents.size() - offset);
            if (written < 0) {
                if (errno == EINTR) continue;
                error = "cannot write output " + path_ + ": " +
                    std::strerror(errno);
                return false;
            }
            if (written == 0) {
                error = "zero-length write for output " + path_;
                return false;
            }
            offset += static_cast<std::size_t>(written);
        }
        return true;
    }

    bool closeForCommit(std::string& error) {
        if (fd_ < 0) return true;
        const int result = ::close(fd_);
        fd_ = -1;
        if (result != 0) {
            error = "cannot close output " + path_ + ": " +
                std::strerror(errno);
            return false;
        }
        return true;
    }

    void preserve() { preserved_ = true; }

private:
    int fd_ = -1;
    std::string path_;
    bool preserved_ = false;
};

bool parseSeed(const std::string& text,
               std::uint32_t& seed,
               std::string& error) {
    try {
        if (text.empty() || text.front() == '-') {
            throw std::invalid_argument("negative or empty seed");
        }
        std::size_t parsed = 0;
        const unsigned long long value = std::stoull(text, &parsed, 10);
        if (parsed != text.size() ||
            value > std::numeric_limits<std::uint32_t>::max()) {
            throw std::out_of_range("seed outside uint32 range");
        }
        seed = static_cast<std::uint32_t>(value);
        return true;
    } catch (const std::exception&) {
        error = "SEED must be an unsigned 32-bit integer";
        return false;
    }
}

std::uint64_t countControlSamples(double duration_sec,
                                  double control_rate_hz) {
    return static_cast<std::uint64_t>(std::ceil(
        duration_sec * control_rate_hz - kCountEpsilon));
}

double firstControlTickAtOrAfter(double time_sec,
                                 double control_rate_hz) {
    const double cycle = std::ceil(
        time_sec * control_rate_hz - kCountEpsilon);
    return cycle / control_rate_hz;
}

simulation::IndependentPlantCommand smokeCommand(double time_sec) {
    simulation::IndependentPlantCommand command;
    if (time_sec >= 0.5 && time_sec < 2.0) {
        command.linear = 0.15;
    } else if (time_sec >= 3.0 && time_sec < 5.0) {
        command.angular = 0.30;
    } else if (time_sec >= 5.5 && time_sec < 7.5) {
        command.linear = 0.12;
        command.angular = -0.35;
    }
    return command;
}

double percentile(std::vector<double> values, double probability) {
    if (values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double index = probability * static_cast<double>(values.size() - 1);
    const std::size_t low = static_cast<std::size_t>(std::floor(index));
    const std::size_t high = std::min(values.size() - 1, low + 1);
    const double fraction = index - static_cast<double>(low);
    return values[low] + fraction * (values[high] - values[low]);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::cerr << "usage: " << argv[0]
                  << " CONFIG_YAML OUTPUT_PREFIX SEED\n";
        return 2;
    }
    const std::string config_path = argv[1];
    const std::string output_prefix = argv[2];
    std::uint32_t seed = 0;
    std::string error;
    if (!parseSeed(argv[3], seed, error)) {
        std::cerr << error << '\n';
        return 2;
    }
    simulation::IndependentPlantConfig config;
    if (!simulation::loadIndependentPlantConfig(
            config_path, config, error)) {
        std::cerr << "config rejected: " << error << '\n';
        return 3;
    }
    simulation::IndependentScoutLiquidPlant plant;
    if (!plant.configure(config, error) || !plant.reset(seed, error)) {
        std::cerr << "plant configure/reset failed: " << error << '\n';
        return 4;
    }

    const std::string csv_path = output_prefix + ".csv";
    const std::string summary_path = output_prefix + ".json";
    ExclusiveOutputFile csv_file;
    ExclusiveOutputFile summary_file;
    if (!csv_file.reserve(csv_path, error) ||
        !summary_file.reserve(summary_path, error)) {
        std::cerr << error << '\n';
        return 5;
    }
    std::ostringstream csv;
    csv << std::setprecision(17);
    csv << "publish_time_sec,sample_time_sec,cmd_v,cmd_omega,"
           "linear_effective_time_sec,angular_effective_time_sec,"
           "linear_transport_jitter_sec,angular_transport_jitter_sec,"
           "active_v,active_omega,x,y,yaw,v,omega,"
           "acceleration,lateral_acceleration,primary_eta_x,primary_eta_y,"
           "second_eta_x,second_eta_y,true_height_m,measured_height_m\n";

    const double control_dt = 1.0 / config.experiment_control_rate_hz;
    const double zero_command_publish_sec = firstControlTickAtOrAfter(
        kMotionCommandEndSec, config.experiment_control_rate_hz);
    const double tail_window_start_sec = zero_command_publish_sec +
        std::max(config.linear.delay_sec, config.angular.delay_sec) +
        config.command_transport_jitter_limit_sec;
    const double end_time = tail_window_start_sec +
        config.experiment_fixed_tail_sec;
    const double raw_sample_count =
        end_time * config.experiment_control_rate_hz;
    if (!std::isfinite(raw_sample_count) || raw_sample_count <= 0.0 ||
        raw_sample_count >
            static_cast<double>(std::numeric_limits<std::uint64_t>::max())) {
        std::cerr << "invalid smoke duration/sample count\n";
        return 5;
    }
    const std::uint64_t sample_count = countControlSamples(
        end_time, config.experiment_control_rate_hz);
    std::vector<double> heights;
    std::vector<double> tail_heights;
    heights.reserve(static_cast<std::size_t>(sample_count));
    tail_heights.reserve(static_cast<std::size_t>(std::ceil(
        config.experiment_fixed_tail_sec *
        config.experiment_control_rate_hz)) + 1u);
    double peak_height = 0.0;
    double max_abs_v = 0.0;
    double max_abs_omega = 0.0;
    for (std::uint64_t cycle = 0; cycle < sample_count; ++cycle) {
        const double time_sec = static_cast<double>(cycle) * control_dt;
        const simulation::IndependentPlantCommand command =
            smokeCommand(time_sec);
        simulation::IndependentPlantPublishReceipt publish_receipt;
        if (!plant.publishCommand(
                time_sec, command, publish_receipt, error) ||
            !plant.advanceTo(
                std::min(end_time, time_sec + control_dt), error)) {
            std::cerr << "plant step failed at cycle " << cycle
                      << ": " << error << '\n';
            return 6;
        }
        const simulation::IndependentPlantState& state = plant.state();
        const simulation::IndependentPlantCommand active =
            plant.activeDelayedCommand();
        csv << publish_receipt.publish_time_sec << ',' << state.time_sec << ','
            << command.linear << ',' << command.angular << ','
            << publish_receipt.linear_effective_time_sec << ','
            << publish_receipt.angular_effective_time_sec << ','
            << publish_receipt.linear_transport_jitter_sec << ','
            << publish_receipt.angular_transport_jitter_sec << ','
            << active.linear << ','
            << active.angular << ',' << state.x << ',' << state.y << ','
            << state.yaw << ',' << state.v << ',' << state.omega << ','
            << state.acceleration << ',' << state.lateral_acceleration << ','
            << state.primary_eta_x << ',' << state.primary_eta_y << ','
            << state.second_eta_x << ',' << state.second_eta_y << ','
            << state.true_height_m << ',' << state.measured_height_m << '\n';
        heights.push_back(state.true_height_m);
        if (state.time_sec + kCountEpsilon >= tail_window_start_sec) {
            tail_heights.push_back(state.true_height_m);
        }
        peak_height = std::max(peak_height, state.true_height_m);
        max_abs_v = std::max(max_abs_v, std::abs(state.v));
        max_abs_omega = std::max(max_abs_omega, std::abs(state.omega));
    }
    if (heights.empty() || tail_heights.empty()) {
        std::cerr << "smoke produced an empty metric window\n";
        return 6;
    }

    const double rms = std::sqrt(std::inner_product(
        heights.begin(), heights.end(), heights.begin(), 0.0) /
        static_cast<double>(heights.size()));
    const double tail_rms = std::sqrt(std::inner_product(
        tail_heights.begin(), tail_heights.end(), tail_heights.begin(), 0.0) /
        static_cast<double>(tail_heights.size()));
    std::ostringstream summary;
    summary << std::setprecision(17);
    summary << "{\n"
            << "  \"schema\": \"spmpc_independent_plant_smoke_v3\",\n"
            << "  \"csv_timestamp_contract\": "
               "\"publish_sample_effective_epochs_v1\",\n"
            << "  \"freeze_id\": \"" << config.freeze_id << "\",\n"
            << "  \"seed\": " << seed << ",\n"
            << "  \"simulation_only\": true,\n"
            << "  \"formal_method_comparison\": false,\n"
            << "  \"control_rate_hz\": "
            << config.experiment_control_rate_hz << ",\n"
            << "  \"motion_command_end_sec\": "
            << kMotionCommandEndSec << ",\n"
            << "  \"zero_command_publish_sec\": "
            << zero_command_publish_sec << ",\n"
            << "  \"tail_window_start_sec\": "
            << tail_window_start_sec << ",\n"
            << "  \"tail_window_end_sec\": " << end_time << ",\n"
            << "  \"end_sec\": " << end_time << ",\n"
            << "  \"samples\": " << heights.size() << ",\n"
            << "  \"tail_samples\": " << tail_heights.size() << ",\n"
            << "  \"duration_sec\": " << plant.state().time_sec << ",\n"
            << "  \"external_height_q95_m\": "
            << percentile(heights, 0.95) << ",\n"
            << "  \"external_height_peak_m\": " << peak_height << ",\n"
            << "  \"external_height_rms_m\": " << rms << ",\n"
            << "  \"tail_height_rms_m\": " << tail_rms << ",\n"
            << "  \"max_abs_v_mps\": " << max_abs_v << ",\n"
            << "  \"max_abs_omega_radps\": " << max_abs_omega << "\n"
            << "}\n";
    if (!csv_file.writeContents(csv.str(), error) ||
        !summary_file.writeContents(summary.str(), error) ||
        !csv_file.closeForCommit(error) ||
        !summary_file.closeForCommit(error)) {
        std::cerr << error << '\n';
        return 7;
    }
    csv_file.preserve();
    summary_file.preserve();
    std::cout << "freeze_id=" << config.freeze_id
              << " seed=" << seed
              << " height_q95_m=" << percentile(heights, 0.95)
              << " peak_m=" << peak_height
              << " tail_rms_m=" << tail_rms << '\n';
    return 0;
}
