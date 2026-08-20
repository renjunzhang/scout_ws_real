#include <gtest/gtest.h>

#include "spmpc_local_planner/dynamics/slosh_dynamics.h"

#include <cstdint>
#include <cstring>

namespace spmpc_local_planner {
namespace {

std::uint64_t doubleBits(double value) {
    std::uint64_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "unexpected double width");
    std::memcpy(&bits, &value, sizeof(bits));
    return bits;
}

struct FrozenDynamicsSnapshot {
    std::uint64_t omega_n;
    std::uint64_t height_coeff;
    std::uint64_t ad_00;
    std::uint64_t ad_10;
    std::uint64_t ad_01;
    std::uint64_t ad_11;
    std::uint64_t bd_00;
    std::uint64_t bd_10;
};

FrozenDynamicsSnapshot snapshot(const SloshDynamics& dynamics) {
    SloshState position_basis;
    position_basis.eta_x = 1.0;
    const SloshState position =
        dynamics.step(position_basis, 0.0, 0.0, 0.0);

    SloshState velocity_basis;
    velocity_basis.eta_x_dot = 1.0;
    const SloshState velocity =
        dynamics.step(velocity_basis, 0.0, 0.0, 0.0);

    const SloshState input =
        dynamics.step(SloshState{}, 1.0, 0.0, 0.0);
    return {
        doubleBits(dynamics.omegaN()),
        doubleBits(dynamics.heightCoeff()),
        doubleBits(position.eta_x),
        doubleBits(position.eta_x_dot),
        doubleBits(velocity.eta_x),
        doubleBits(velocity.eta_x_dot),
        doubleBits(input.eta_x),
        doubleBits(input.eta_x_dot),
    };
}

void expectSnapshot(const FrozenDynamicsSnapshot& actual,
                    const FrozenDynamicsSnapshot& expected) {
    EXPECT_EQ(actual.omega_n, expected.omega_n);
    EXPECT_EQ(actual.height_coeff, expected.height_coeff);
    EXPECT_EQ(actual.ad_00, expected.ad_00);
    EXPECT_EQ(actual.ad_10, expected.ad_10);
    EXPECT_EQ(actual.ad_01, expected.ad_01);
    EXPECT_EQ(actual.ad_11, expected.ad_11);
    EXPECT_EQ(actual.bd_00, expected.bd_00);
    EXPECT_EQ(actual.bd_10, expected.bd_10);
}

void expectDecoupledAxes(const SloshDynamics& dynamics,
                         const FrozenDynamicsSnapshot& expected) {
    SloshState y_position_basis;
    y_position_basis.eta_y = 1.0;
    const SloshState y_position =
        dynamics.step(y_position_basis, 0.0, 0.0, 0.0);
    EXPECT_EQ(doubleBits(y_position.eta_y), expected.ad_00);
    EXPECT_EQ(doubleBits(y_position.eta_y_dot), expected.ad_10);
    EXPECT_EQ(y_position.eta_x, 0.0);
    EXPECT_EQ(y_position.eta_x_dot, 0.0);

    SloshState y_velocity_basis;
    y_velocity_basis.eta_y_dot = 1.0;
    const SloshState y_velocity =
        dynamics.step(y_velocity_basis, 0.0, 0.0, 0.0);
    EXPECT_EQ(doubleBits(y_velocity.eta_y), expected.ad_01);
    EXPECT_EQ(doubleBits(y_velocity.eta_y_dot), expected.ad_11);
    EXPECT_EQ(y_velocity.eta_x, 0.0);
    EXPECT_EQ(y_velocity.eta_x_dot, 0.0);

    const SloshState y_input =
        dynamics.step(SloshState{}, 0.0, 1.0, 0.0);
    EXPECT_EQ(doubleBits(y_input.eta_y), expected.bd_00);
    EXPECT_EQ(doubleBits(y_input.eta_y_dot), expected.bd_10);
    EXPECT_EQ(y_input.eta_x, 0.0);
    EXPECT_EQ(y_input.eta_x_dot, 0.0);
}

TEST(SloshDynamics, PreservesFrozenDefaultModalAndZohContractBitwise) {
    SloshDynamics dynamics;
    ASSERT_TRUE(dynamics.configure(SloshModelParams{}));

    const FrozenDynamicsSnapshot expected = {
        0x403f3efc27a80c67ULL,
        0x3ffd164837f0714cULL,
        0x3fe0aec01c58f364ULL,
        0xc0399d66a514fbcaULL,
        0x3f9addadab9a2180ULL,
        0x3fdc1e5feebf5c41ULL,
        0xbf4010c591d5cecdULL,
        0xbf9addadab9a214cULL,
    };
    expectSnapshot(snapshot(dynamics), expected);
    expectDecoupledAxes(dynamics, expected);
}

TEST(SloshDynamics, PreservesFrozenHigherModeNonlinearContractBitwise) {
    SloshModelParams params;
    params.mode_index = 3;
    params.damping_ratio = 0.17;
    params.dt = 0.017;
    params.container_radius = 0.031;
    params.liquid_height = 0.044;
    params.liquid_density = 850.0;
    params.use_linear_model = false;

    SloshDynamics dynamics;
    ASSERT_TRUE(dynamics.configure(params));
    const FrozenDynamicsSnapshot expected = {
        0x4049fcb53ecf9914ULL,
        0x3fce6827df326731ULL,
        0x3fe55ff667577130ULL,
        0xc0415af0005b7fe4ULL,
        0x3f8a50cadfc9daf6ULL,
        0x3fdc37b0273643d7ULL,
        0xbf201c5f4a85dc6aULL,
        0xbf8a50cadfc9d951ULL,
    };
    expectSnapshot(snapshot(dynamics), expected);
    expectDecoupledAxes(dynamics, expected);
}

TEST(SloshDynamics, PreservesHistoricalConfigurationAdmission) {
    SloshModelParams params;
    SloshDynamics dynamics;

    params.container_radius = 0.0;
    EXPECT_FALSE(dynamics.configure(params));
    params = SloshModelParams{};
    params.liquid_height = 0.0;
    EXPECT_FALSE(dynamics.configure(params));
    params = SloshModelParams{};
    params.liquid_density = 0.0;
    EXPECT_FALSE(dynamics.configure(params));
    params = SloshModelParams{};
    params.dt = 1e-4;
    EXPECT_FALSE(dynamics.configure(params));
    params = SloshModelParams{};
    params.mode_index = 0;
    EXPECT_FALSE(dynamics.configure(params));
    params.mode_index = 6;
    EXPECT_FALSE(dynamics.configure(params));

    params = SloshModelParams{};
    params.damping_ratio = -0.1;
    EXPECT_TRUE(dynamics.configure(params));
    params.damping_ratio = 1.1;
    EXPECT_TRUE(dynamics.configure(params));
}

}  // namespace
}  // namespace spmpc_local_planner

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
