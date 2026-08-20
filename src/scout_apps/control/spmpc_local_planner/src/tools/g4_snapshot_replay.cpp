#include "spmpc_local_planner/analysis/g4_snapshot_replay.h"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace replay = spmpc_local_planner::analysis;

namespace {

constexpr std::size_t kMaximumFrames = 100000;
constexpr std::size_t kMaximumValues = 100000000;

void expect(std::istream& input, const std::string& expected) {
    std::string token;
    if (!(input >> token) || token != expected) {
        throw std::runtime_error(
            "expected token '" + expected + "', received '" + token + "'");
    }
}

template <typename T>
T readNamed(std::istream& input, const std::string& name) {
    expect(input, name);
    T value{};
    if (!(input >> value)) {
        throw std::runtime_error("failed to read " + name);
    }
    return value;
}

std::vector<double> readVector(
    std::istream& input, const std::string& name) {
    expect(input, name);
    std::size_t count = 0;
    if (!(input >> count) || count > kMaximumValues) {
        throw std::runtime_error("invalid vector count for " + name);
    }
    std::vector<double> values(count, 0.0);
    for (double& value : values) {
        if (!(input >> value)) {
            throw std::runtime_error("truncated vector " + name);
        }
    }
    return values;
}

std::vector<replay::G4ReplayFrame> readRequest(std::istream& input) {
    expect(input, "SPMPC_G4_SNAPSHOT_REPLAY_V1");
    const std::size_t frame_count =
        readNamed<std::size_t>(input, "frames");
    if (frame_count == 0 || frame_count > kMaximumFrames) {
        throw std::runtime_error("invalid frame count");
    }
    std::vector<replay::G4ReplayFrame> frames;
    frames.reserve(frame_count);
    for (std::size_t frame_index = 0;
         frame_index < frame_count; ++frame_index) {
        expect(input, "frame");
        replay::G4ReplayFrame frame;
        frame.pair_index = readNamed<int>(input, "pair_index");
        frame.direction_code = readNamed<int>(input, "direction");
        expect(input, "dimensions");
        if (!(input >> frame.horizon_steps >> frame.state_width >>
              frame.control_width >> frame.parameter_width)) {
            throw std::runtime_error("failed to read dimensions");
        }
        frame.dt = readNamed<double>(input, "dt");
        expect(input, "initial_state");
        for (double& value : frame.initial_state) {
            if (!(input >> value)) {
                throw std::runtime_error("truncated initial_state");
            }
        }
        expect(input, "bounds");
        if (!(input >> frame.runtime_bounds.a_min >>
              frame.runtime_bounds.a_max >>
              frame.runtime_bounds.alpha_min >>
              frame.runtime_bounds.alpha_max >>
              frame.runtime_bounds.v_s_min >>
              frame.runtime_bounds.v_s_max >>
              frame.runtime_bounds.v_min >>
              frame.runtime_bounds.v_max >>
              frame.runtime_bounds.omega_min >>
              frame.runtime_bounds.omega_max)) {
            throw std::runtime_error("truncated runtime bounds");
        }
        frame.stage_parameters = readVector(input, "stage_parameters");
        frame.initial_guess_states =
            readVector(input, "initial_guess_states");
        frame.initial_guess_controls =
            readVector(input, "initial_guess_controls");
        expect(input, "modal_overrides");
        std::size_t override_count = 0;
        if (!(input >> override_count) || override_count > 1000) {
            throw std::runtime_error("invalid modal override count");
        }
        frame.modal_overrides.resize(override_count);
        for (auto& modal : frame.modal_overrides) {
            for (double& value : modal) {
                if (!(input >> value)) {
                    throw std::runtime_error("truncated modal override");
                }
            }
        }
        expect(input, "end_frame");
        frames.push_back(std::move(frame));
    }
    expect(input, "end_request");
    return frames;
}

void writeVector(std::ostream& output,
                 const std::string& name,
                 const std::vector<double>& values) {
    output << name << ' ' << values.size();
    for (double value : values) output << ' ' << value;
    output << '\n';
}

void writeSolution(std::ostream& output,
                   const replay::G4ReplaySolution& solution) {
    output << "solution " << solution.status << '\n';
    writeVector(output, "states", solution.states);
    writeVector(output, "controls", solution.controls);
    output << "end_solution\n";
}

void writeResult(std::ostream& output,
                 const replay::G4SequenceReplayResult& result) {
    output << std::setprecision(std::numeric_limits<double>::max_digits10);
    output << "SPMPC_G4_SNAPSHOT_REPLAY_RESULT_V1\n";
    output << "success " << (result.success ? 1 : 0) << '\n';
    output << "detail " << result.detail << '\n';
    output << "failed_pair " << result.failed_pair_index << '\n';
    output << "checkpoints " << result.checkpoints.size() << '\n';
    for (const auto& checkpoint : result.checkpoints) {
        output << "checkpoint\n";
        output << "pair_index " << checkpoint.pair_index << '\n';
        output << "direction " << checkpoint.direction_code << '\n';
        output << "actual\n";
        writeSolution(output, checkpoint.actual);
        output << "counterfactuals "
               << checkpoint.counterfactuals.size() << '\n';
        for (const auto& solution : checkpoint.counterfactuals) {
            writeSolution(output, solution);
        }
        output << "end_checkpoint\n";
    }
    output << "end_result\n";
}

}  // namespace

int main(int argc, char** argv) {
    std::string input_path;
    std::string output_path;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--input" && index + 1 < argc) {
            input_path = argv[++index];
        } else if (argument == "--output" && index + 1 < argc) {
            output_path = argv[++index];
        } else {
            std::cerr << "usage: spmpc_g4_snapshot_replay "
                         "--input REQUEST --output RESULT\n";
            return 2;
        }
    }
    if (input_path.empty() || output_path.empty()) {
        std::cerr << "both --input and --output are required\n";
        return 2;
    }
    try {
        std::ifstream input(input_path);
        if (!input) throw std::runtime_error("cannot open replay request");
        const std::vector<replay::G4ReplayFrame> frames =
            readRequest(input);
        const replay::G4SequenceReplayResult result =
            replay::G4SnapshotReplayRunner::run(frames);
        std::ofstream output(output_path);
        if (!output) throw std::runtime_error("cannot open replay result");
        writeResult(output, result);
        if (!output) throw std::runtime_error("failed to write replay result");
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "spmpc_g4_snapshot_replay: "
                  << exception.what() << '\n';
        return 2;
    }
}
