#include "spmpc_local_planner/tools/short_horizon_matched_preflight.h"

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

int main(int argc, char** argv) {
    std::string input_path;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--input" && index + 1 < argc) {
            input_path = argv[++index];
        } else {
            std::cerr << "usage: spmpc_short_horizon_matched_preflight "
                         "[--input PARAM_DUMP]\n";
            return 2;
        }
    }
    std::ostringstream buffer;
    if (input_path.empty()) {
        buffer << std::cin.rdbuf();
    } else {
        std::ifstream input(input_path);
        if (!input) {
            std::cerr << "[SHORT-HORIZON-MATCHED-PREFLIGHT] cannot open "
                      << input_path << '\n';
            return 2;
        }
        buffer << input.rdbuf();
    }
    const auto result =
        spmpc_local_planner::tools::validateShortHorizonMatchedParamDump(
            buffer.str());
    if (!result.success) {
        std::cerr << "[SHORT-HORIZON-MATCHED-PREFLIGHT] FAIL: "
                  << result.detail << '\n';
        return 1;
    }
    std::cout << "[SHORT-HORIZON-MATCHED-PREFLIGHT] PASS\n";
    return 0;
}
