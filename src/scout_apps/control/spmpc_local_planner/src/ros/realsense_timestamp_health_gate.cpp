#include "spmpc_local_planner/runtime/timestamp_health_evaluator.h"

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <pwd.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace spmpc_local_planner {
namespace {

struct GateOptions {
    std::string topic = "/camera/color/camera_info";
    int samples = 90;
    double timeout_sec = 10.0;
    TimestampHealthThresholds thresholds;
    bool settle_until_pass = false;
    std::string report;
};

struct GateReportContext {
    std::string topic;
    int requested_samples = 0;
    double timeout_sec = 0.0;
    bool settle_until_pass = false;
    std::size_t observed_samples = 0;
    std::size_t windows_evaluated = 0;
    bool timed_out = false;
};

void printUsage(std::ostream& stream) {
    stream
        << "usage: spmpc_realsense_timestamp_health_gate [options] --report PATH\n"
        << "\nFail-closed pre-motion health gate for a stamped RealSense ROS topic.\n\n"
        << "  --topic TOPIC\n"
        << "  --samples COUNT\n"
        << "  --timeout-sec SEC\n"
        << "  --max-future-skew-sec SEC\n"
        << "  --max-p95-lag-sec SEC\n"
        << "  --min-clock-rate-ratio RATIO\n"
        << "  --max-clock-rate-ratio RATIO\n"
        << "  --max-gap-sec SEC\n"
        << "  --settle-until-pass\n"
        << "  --report PATH\n";
}

bool splitArgument(const std::string& argument,
                   std::string& name,
                   std::string& value) {
    const std::size_t equals = argument.find('=');
    if (equals == std::string::npos) {
        name = argument;
        value.clear();
        return false;
    }
    name = argument.substr(0, equals);
    value = argument.substr(equals + 1);
    return true;
}

double parseDouble(const std::string& value, const std::string& name) {
    std::size_t parsed = 0;
    try {
        const double output = std::stod(value, &parsed);
        if (parsed != value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return output;
    } catch (const std::exception&) {
        throw std::invalid_argument(name + " expects a floating-point value");
    }
}

int parseInteger(const std::string& value, const std::string& name) {
    std::size_t parsed = 0;
    try {
        const int output = std::stoi(value, &parsed);
        if (parsed != value.size()) {
            throw std::invalid_argument("trailing characters");
        }
        return output;
    } catch (const std::exception&) {
        throw std::invalid_argument(name + " expects an integer value");
    }
}

bool parseOptions(int argc, char** argv, GateOptions& options, int& exit_code) {
    exit_code = 2;
    for (int index = 1; index < argc; ++index) {
        std::string name;
        std::string value;
        const bool inline_value = splitArgument(argv[index], name, value);
        if (name == "--help" || name == "-h") {
            printUsage(std::cout);
            exit_code = 0;
            return false;
        }
        if (name == "--settle-until-pass") {
            if (inline_value) {
                throw std::invalid_argument(
                    "--settle-until-pass does not accept a value");
            }
            options.settle_until_pass = true;
            continue;
        }
        if (name.empty() || name.rfind("--", 0) != 0) {
            throw std::invalid_argument("unrecognized argument: " + name);
        }
        if (!inline_value) {
            if (index + 1 >= argc) {
                throw std::invalid_argument(name + " requires a value");
            }
            value = argv[++index];
        }
        if (name == "--topic") {
            options.topic = value;
        } else if (name == "--samples") {
            options.samples = parseInteger(value, name);
        } else if (name == "--timeout-sec") {
            options.timeout_sec = parseDouble(value, name);
        } else if (name == "--max-future-skew-sec") {
            options.thresholds.max_future_skew_sec = parseDouble(value, name);
        } else if (name == "--max-p95-lag-sec") {
            options.thresholds.max_p95_lag_sec = parseDouble(value, name);
        } else if (name == "--min-clock-rate-ratio") {
            options.thresholds.min_clock_rate_ratio = parseDouble(value, name);
        } else if (name == "--max-clock-rate-ratio") {
            options.thresholds.max_clock_rate_ratio = parseDouble(value, name);
        } else if (name == "--max-gap-sec") {
            options.thresholds.max_gap_sec = parseDouble(value, name);
        } else if (name == "--report") {
            options.report = value;
        } else {
            throw std::invalid_argument("unrecognized argument: " + name);
        }
    }
    if (options.report.empty()) {
        throw std::invalid_argument("--report is required");
    }
    return true;
}

std::string jsonEscape(const std::string& input) {
    std::ostringstream stream;
    stream << '"';
    for (const unsigned char character : input) {
        switch (character) {
            case '"': stream << "\\\""; break;
            case '\\': stream << "\\\\"; break;
            case '\b': stream << "\\b"; break;
            case '\f': stream << "\\f"; break;
            case '\n': stream << "\\n"; break;
            case '\r': stream << "\\r"; break;
            case '\t': stream << "\\t"; break;
            default:
                if (character < 0x20) {
                    stream << "\\u" << std::hex << std::setw(4)
                           << std::setfill('0')
                           << static_cast<int>(character) << std::dec;
                } else {
                    stream << static_cast<char>(character);
                }
        }
    }
    stream << '"';
    return stream.str();
}

std::string jsonNumber(double value) {
    if (std::isnan(value)) return "NaN";
    if (value == std::numeric_limits<double>::infinity()) return "Infinity";
    if (value == -std::numeric_limits<double>::infinity()) return "-Infinity";
    std::ostringstream stream;
    stream << std::setprecision(17) << value;
    return stream.str();
}

std::string jsonFailures(const std::vector<std::string>& failures) {
    if (failures.empty()) return "[]";
    std::ostringstream stream;
    stream << "[\n";
    for (std::size_t index = 0; index < failures.size(); ++index) {
        stream << "    " << jsonEscape(failures[index]);
        if (index + 1 < failures.size()) stream << ',';
        stream << '\n';
    }
    stream << "  ]";
    return stream.str();
}

std::string jsonThresholds(const TimestampHealthThresholds& thresholds) {
    std::map<std::string, std::string> fields;
    fields["max_future_skew_sec"] =
        jsonNumber(thresholds.max_future_skew_sec);
    fields["max_gap_sec"] = jsonNumber(thresholds.max_gap_sec);
    fields["max_p95_lag_sec"] = jsonNumber(thresholds.max_p95_lag_sec);
    fields["max_clock_rate_ratio"] =
        jsonNumber(thresholds.max_clock_rate_ratio);
    fields["min_clock_rate_ratio"] =
        jsonNumber(thresholds.min_clock_rate_ratio);
    std::ostringstream stream;
    stream << "{\n";
    std::size_t index = 0;
    for (const auto& field : fields) {
        stream << "    " << jsonEscape(field.first) << ": " << field.second;
        if (++index < fields.size()) stream << ',';
        stream << '\n';
    }
    stream << "  }";
    return stream.str();
}

std::string renderReport(const TimestampHealthResult& result,
                         const GateReportContext& context) {
    std::map<std::string, std::string> fields;
    fields["failures"] = jsonFailures(result.failures);
    fields["nonfinite_samples"] =
        std::to_string(result.nonfinite_samples);
    fields["observed_samples"] = std::to_string(context.observed_samples);
    fields["requested_samples"] =
        std::to_string(context.requested_samples);
    fields["samples"] = std::to_string(result.samples);
    fields["schema_version"] = "1";
    fields["settle_until_pass"] =
        context.settle_until_pass ? "true" : "false";
    fields["status"] = jsonEscape(result.status);
    fields["timed_out"] = context.timed_out ? "true" : "false";
    fields["timeout_sec"] = jsonNumber(context.timeout_sec);
    fields["topic"] = jsonEscape(context.topic);
    fields["windows_evaluated"] =
        std::to_string(context.windows_evaluated);
    if (result.has_zero_stamp_samples) {
        fields["zero_stamp_samples"] =
            std::to_string(result.zero_stamp_samples);
    }
    if (result.has_full_metrics) {
        fields["clock_rate_ratio"] = jsonNumber(result.clock_rate_ratio);
        fields["max_receive_minus_source_sec"] =
            jsonNumber(result.max_receive_minus_source_sec);
        fields["max_source_gap_sec"] =
            jsonNumber(result.max_source_gap_sec);
        fields["min_receive_minus_source_sec"] =
            jsonNumber(result.min_receive_minus_source_sec);
        fields["p95_receive_minus_source_sec"] =
            jsonNumber(result.p95_receive_minus_source_sec);
        fields["positive_stamp_samples"] =
            std::to_string(result.positive_stamp_samples);
        fields["receive_regressions"] =
            std::to_string(result.receive_regressions);
        fields["receive_span_sec"] = jsonNumber(result.receive_span_sec);
        fields["source_regressions"] =
            std::to_string(result.source_regressions);
        fields["source_span_sec"] = jsonNumber(result.source_span_sec);
        fields["thresholds"] = jsonThresholds(result.thresholds);
    }

    std::ostringstream stream;
    stream << "{\n";
    std::size_t index = 0;
    for (const auto& field : fields) {
        stream << "  " << jsonEscape(field.first) << ": " << field.second;
        if (++index < fields.size()) stream << ',';
        stream << '\n';
    }
    stream << "}\n";
    return stream.str();
}

std::string absoluteReportPath(const std::string& input) {
    std::string path = input;
    if (!path.empty() && path[0] == '~' &&
        (path.size() == 1 || path[1] == '/')) {
        const passwd* user = getpwuid(getuid());
        if (user != nullptr && user->pw_dir != nullptr) {
            path = std::string(user->pw_dir) + path.substr(1);
        }
    }
    if (!path.empty() && path[0] == '/') return path;
    char working_directory[4096] = {0};
    if (getcwd(working_directory, sizeof(working_directory)) == nullptr) {
        throw std::runtime_error(
            "cannot resolve report path: " + std::string(std::strerror(errno)));
    }
    return std::string(working_directory) + "/" + path;
}

bool createParentDirectories(const std::string& file_path,
                             std::string& error) {
    const std::size_t final_slash = file_path.find_last_of('/');
    if (final_slash == std::string::npos || final_slash == 0) return true;
    const std::string directory = file_path.substr(0, final_slash);
    for (std::size_t slash = 1; slash <= directory.size(); ++slash) {
        if (slash < directory.size() && directory[slash] != '/') continue;
        const std::string prefix = directory.substr(0, slash);
        if (prefix.empty()) continue;
        if (mkdir(prefix.c_str(), 0755) != 0 && errno != EEXIST) {
            error = "cannot create report directory " + prefix + ": " +
                std::strerror(errno);
            return false;
        }
    }
    return true;
}

bool writeReport(const std::string& path,
                 const std::string& contents,
                 std::string& error) {
    if (!createParentDirectories(path, error)) return false;
    std::ofstream stream(path);
    if (!stream) {
        error = "cannot open report for writing: " + path;
        return false;
    }
    stream << contents;
    if (!stream) {
        error = "failed while writing report: " + path;
        return false;
    }
    return true;
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    using namespace spmpc_local_planner;
    GateOptions options;
    int parse_exit_code = 2;
    try {
        if (!parseOptions(argc, argv, options, parse_exit_code)) {
            return parse_exit_code;
        }
    } catch (const std::exception& error) {
        std::cerr << "spmpc_realsense_timestamp_health_gate: error: "
                  << error.what() << '\n';
        printUsage(std::cerr);
        return 2;
    }
    if (options.samples < 2 || options.timeout_sec <= 0.0) {
        std::cerr << "[RealSense timestamp gate] invalid samples/timeout\n";
        return 2;
    }

    ros::init(argc, argv, "realsense_timestamp_health_gate",
              ros::init_options::AnonymousName |
              ros::init_options::NoSigintHandler);
    ros::NodeHandle node;
    std::vector<TimestampRecord> records;
    std::mutex mutex;
    std::condition_variable changed;
    bool complete = false;
    std::unique_ptr<TimestampHealthResult> passing_result;
    std::size_t windows_evaluated = 0;

    ros::Subscriber subscriber = node.subscribe<sensor_msgs::CameraInfo>(
        options.topic, 1,
        [&](const sensor_msgs::CameraInfo::ConstPtr& message) {
            std::lock_guard<std::mutex> lock(mutex);
            if (complete) return;
            records.push_back({
                ros::Time::now().toSec(), message->header.stamp.toSec()});
            if (records.size() < static_cast<std::size_t>(options.samples)) {
                return;
            }
            if (!options.settle_until_pass) {
                complete = true;
                changed.notify_all();
                return;
            }
            ++windows_evaluated;
            const std::vector<TimestampRecord> window(
                records.end() - options.samples, records.end());
            TimestampHealthResult candidate =
                TimestampHealthEvaluator::evaluate(
                    window, options.thresholds);
            if (candidate.status == "PASS") {
                passing_result.reset(
                    new TimestampHealthResult(std::move(candidate)));
                complete = true;
                changed.notify_all();
            }
        });
    ros::AsyncSpinner spinner(1);
    spinner.start();

    const auto deadline = std::chrono::steady_clock::now() +
        std::chrono::duration<double>(options.timeout_sec);
    {
        std::unique_lock<std::mutex> lock(mutex);
        while (!complete && std::chrono::steady_clock::now() < deadline &&
               ros::ok()) {
            changed.wait_for(lock, std::chrono::milliseconds(50));
        }
    }
    subscriber.shutdown();
    spinner.stop();

    std::vector<TimestampRecord> snapshot;
    std::unique_ptr<TimestampHealthResult> accepted_result;
    std::size_t evaluated_count = 0;
    bool timed_out = false;
    {
        std::lock_guard<std::mutex> lock(mutex);
        snapshot = records;
        if (passing_result) {
            accepted_result.reset(
                new TimestampHealthResult(*passing_result));
        }
        evaluated_count = windows_evaluated;
        timed_out = !complete;
    }

    TimestampHealthResult result;
    if (accepted_result) {
        result = *accepted_result;
    } else {
        const std::size_t start = snapshot.size() >
                static_cast<std::size_t>(options.samples)
            ? snapshot.size() - static_cast<std::size_t>(options.samples)
            : 0;
        const std::vector<TimestampRecord> window(
            snapshot.begin() + static_cast<std::ptrdiff_t>(start),
            snapshot.end());
        result = TimestampHealthEvaluator::evaluate(
            window, options.thresholds);
        if (snapshot.size() >= static_cast<std::size_t>(options.samples)) {
            evaluated_count = std::max<std::size_t>(evaluated_count, 1);
        }
    }
    if (snapshot.size() < static_cast<std::size_t>(options.samples)) {
        result.status = "FAIL";
        result.failures.push_back(
            "received " + std::to_string(snapshot.size()) + " of " +
            std::to_string(options.samples) + " required samples");
    } else if (options.settle_until_pass && !accepted_result) {
        result.status = "FAIL";
        std::ostringstream failure;
        failure << "no healthy consecutive " << options.samples
                << "-sample window within " << std::fixed
                << std::setprecision(1) << options.timeout_sec << 's';
        result.failures.push_back(failure.str());
    }

    GateReportContext report_context;
    report_context.topic = options.topic;
    report_context.requested_samples = options.samples;
    report_context.timeout_sec = options.timeout_sec;
    report_context.settle_until_pass = options.settle_until_pass;
    report_context.observed_samples = snapshot.size();
    report_context.windows_evaluated = evaluated_count;
    report_context.timed_out = timed_out;

    std::string report_path;
    try {
        report_path = absoluteReportPath(options.report);
    } catch (const std::exception& error) {
        std::cerr << "[RealSense timestamp gate] " << error.what() << '\n';
        return 2;
    }
    std::string write_error;
    if (!writeReport(
            report_path, renderReport(result, report_context), write_error)) {
        std::cerr << "[RealSense timestamp gate] " << write_error << '\n';
        return 2;
    }

    std::cout << "[RealSense timestamp gate] " << result.status
              << ": samples=" << result.samples
              << "; min_lag=" << std::fixed << std::setprecision(4)
              << result.min_receive_minus_source_sec << "s; p95_lag="
              << result.p95_receive_minus_source_sec
              << "s; rate_ratio=" << std::setprecision(6)
              << result.clock_rate_ratio << "; max_gap="
              << std::setprecision(4) << result.max_source_gap_sec << "s\n"
              << "  report = " << report_path << '\n';
    for (const std::string& failure : result.failures) {
        std::cerr << "  FAIL: " << failure << '\n';
    }
    return result.status == "PASS" ? 0 : 1;
}
