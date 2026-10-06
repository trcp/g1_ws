//
// SPDX-FileCopyrightText: Copyright 2024-2025 Arm Limited and/or its affiliates <open-source-office@arm.com>
//
// SPDX-License-Identifier: Apache-2.0
//

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <sstream>
#include <string>
#include <tuple>

#include "kai/ukernels/matmul/matmul_clamp_qai8_qai8_qsi8cxp/kai_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot.h"
#include "kai/ukernels/matmul/matmul_clamp_qai8_qai8_qsi8cxp/kai_matmul_clamp_qai8_qai8_qsi8cxp_interface.h"
#include "kai/ukernels/matmul/matmul_clamp_qai8_qai8p_qsi8cxp/kai_matmul_clamp_qai8_qai8p2vlx4_qsi8cxpsb2vlx4_2vlx2vl_sme2_mopa.h"
#include "kai/ukernels/matmul/pack/kai_lhs_pack_x8p2vlx4_x8_sme.h"
#include "kai/ukernels/matmul/pack/kai_rhs_pack_kxn_qsi8cxp2vlx4sb_qs8cx_f32_i32_sme.h"
#include "test/common/cpu_info.hpp"
#include "test/common/matrix_portion.hpp"
#include "test/common/memory.hpp"
#include "test/common/rect.hpp"
#include "test/common/sme.hpp"
#include "test/common/test_suite.hpp"
#include "test/reference/binary_elementwise.hpp"
#include "test/reference/clamp.hpp"
#include "test/reference/fill.hpp"
#include "test/reference/matmul.hpp"
#include "test/reference/matmul_pack.hpp"
#include "test/reference/quantize.hpp"
#include "test/reference/reduce.hpp"
#include "test/reference/reorder.hpp"
#include "test/reference/transpose.hpp"

namespace kai::test {

using Buffer = std::vector<uint8_t>;

struct LhsPackKernel {
    std::function<size_t(size_t mr)> get_m_step;
    std::function<size_t(size_t m_idx, size_t lhs_stride)> get_lhs_offset;
    std::function<size_t(size_t m_idx, size_t k, size_t mr, size_t kr, size_t sr)> get_packed_lhs_offset;
    std::function<size_t(size_t m, size_t k, size_t mr, size_t kr, size_t sr)> get_packed_lhs_size;
    std::function<void(
        size_t m, size_t k, size_t mr, size_t kr, size_t sr, size_t m_idx_start, const void* lhs, size_t lhs_stride,
        void* lhs_packed)>
        pack;
};

struct RhsPackKernel {
    std::function<size_t()> get_n_step;
    std::function<size_t(size_t n_idx)> get_rhs_offset;
    std::function<size_t(size_t n_idx)> get_bias_offset;
    std::function<size_t(size_t n_idx)> get_scale_offset;
    std::function<size_t(size_t n_idx, size_t k)> get_packed_rhs_offset;
    std::function<size_t(size_t n, size_t k)> get_packed_rhs_size;
    std::function<void(
        size_t num_groups, size_t n, size_t k, size_t nr, size_t kr, size_t sr, size_t rhs_stride, const void* rhs,
        const void* bias, const void* scale, void* rhs_packed, size_t extra_bytes,
        const struct kai_rhs_pack_qsi8cx_params* params)>
        pack;
};

struct MatMulKernel {
    std::function<size_t(void)> get_m_step;
    std::function<size_t(void)> get_n_step;
    std::function<size_t(void)> get_mr;
    std::function<size_t(void)> get_nr;
    std::function<size_t(void)> get_kr;
    std::function<size_t(void)> get_sr;
    std::function<size_t(size_t m_idx, size_t k)> get_packed_lhs_offset;
    std::function<size_t(size_t n_idx, size_t k)> get_packed_rhs_offset;
    std::function<size_t(size_t m_idx, size_t n_idx, size_t dst_stride)> get_dst_offset;
    std::function<size_t(size_t m, size_t n)> get_dst_size;
    std::function<void(
        size_t m, size_t n, size_t k, const void* lhs_packed, const void* rhs_packed, void* dst, size_t dst_stride_row,
        size_t dst_stride_col, const kai_matmul_requantize32_params* params)>
        matmul;
};

const static RhsPackKernel rhs_pack = {
    .get_n_step = kai_get_n_step_rhs_pack_kxn_qsi8cxp2vlx4sb_qs8cx_f32_i32_sme,
    .get_rhs_offset = kai_get_rhs_offset_rhs_pack_kxn_qsi8cxp2vlx4sb_qs8cx_f32_i32_sme,
    .get_bias_offset = kai_get_bias_offset_rhs_pack_kxn_qsi8cxp2vlx4sb_qs8cx_f32_i32_sme,
    .get_scale_offset = kai_get_scale_offset_rhs_pack_kxn_qsi8cxp2vlx4sb_qs8cx_f32_i32_sme,
    .get_packed_rhs_offset = kai_get_rhs_packed_offset_rhs_pack_kxn_qsi8cxp2vlx4sb_qs8cx_f32_i32_sme,
    .get_packed_rhs_size = kai_get_rhs_packed_size_rhs_pack_kxn_qsi8cxp2vlx4sb_qs8cx_f32_i32_sme,
    .pack = kai_run_rhs_pack_kxn_qsi8cxp2vlx4sb_qs8cx_f32_i32_sme,
};

struct MatMulVariant {
    std::string_view name;  ///< Test identification
    MatMulShape acc_pack;   ///< Accumulator shape for packing (mr/nr/kr)
    MatMulShape acc_step;   ///< Accumulator shape for matmul (stepping)

    std::function<bool(void)> is_supported;  ///< HW support check

    std::optional<LhsPackKernel> lhs_pack;  ///< LHS packing kernel interface
    RhsPackKernel rhs_pack;                 ///< RHS packing kernel interface
    MatMulKernel matmul;                    ///< Matmul kernel interface
};

const std::array gemm_variants = {
    MatMulVariant{
        .name = "matmul_qai8_qai8p_qsi8cxp",
        .acc_pack{
            .m = 2 * get_sme_vector_length<int32_t>(),
            .n = 2 * get_sme_vector_length<int32_t>(),
            .k = sizeof(int32_t) / sizeof(int8_t),
        },
        .acc_step{
            .m = 2 * get_sme_vector_length<int32_t>(),
            .n = 2 * get_sme_vector_length<int32_t>(),
            .k = sizeof(int32_t) / sizeof(int8_t),
        },

        .is_supported = cpu_has_sme2,

        .lhs_pack =
            LhsPackKernel{
                .get_m_step = kai_get_m_step_lhs_pack_x8p2vlx4_x8_sme,
                .get_lhs_offset = kai_get_lhs_offset_lhs_pack_x8p2vlx4_x8_sme,
                .get_packed_lhs_offset = kai_get_lhs_packed_offset_lhs_pack_x8p2vlx4_x8_sme,
                .get_packed_lhs_size = kai_get_lhs_packed_size_lhs_pack_x8p2vlx4_x8_sme,
                .pack = kai_run_lhs_pack_x8p2vlx4_x8_sme,
            },
        .rhs_pack = rhs_pack,
        .matmul =
            MatMulKernel{
                .get_m_step = kai_get_m_step_matmul_clamp_qai8_qai8p2vlx4_qsi8cxpsb2vlx4_2vlx2vl_sme2_mopa,
                .get_n_step = kai_get_n_step_matmul_clamp_qai8_qai8p2vlx4_qsi8cxpsb2vlx4_2vlx2vl_sme2_mopa,
                .get_mr = kai_get_mr_matmul_clamp_qai8_qai8p2vlx4_qsi8cxpsb2vlx4_2vlx2vl_sme2_mopa,
                .get_nr = kai_get_nr_matmul_clamp_qai8_qai8p2vlx4_qsi8cxpsb2vlx4_2vlx2vl_sme2_mopa,
                .get_kr = kai_get_kr_matmul_clamp_qai8_qai8p2vlx4_qsi8cxpsb2vlx4_2vlx2vl_sme2_mopa,
                .get_sr = kai_get_sr_matmul_clamp_qai8_qai8p2vlx4_qsi8cxpsb2vlx4_2vlx2vl_sme2_mopa,
                .get_packed_lhs_offset =
                    kai_get_lhs_packed_offset_matmul_clamp_qai8_qai8p2vlx4_qsi8cxpsb2vlx4_2vlx2vl_sme2_mopa,
                .get_packed_rhs_offset =
                    kai_get_rhs_packed_offset_matmul_clamp_qai8_qai8p2vlx4_qsi8cxpsb2vlx4_2vlx2vl_sme2_mopa,
                .get_dst_offset = kai_get_dst_offset_matmul_clamp_qai8_qai8p2vlx4_qsi8cxpsb2vlx4_2vlx2vl_sme2_mopa,
                .get_dst_size = kai_get_dst_size_matmul_clamp_qai8_qai8p2vlx4_qsi8cxpsb2vlx4_2vlx2vl_sme2_mopa,
                .matmul = kai_run_matmul_clamp_qai8_qai8p2vlx4_qsi8cxpsb2vlx4_2vlx2vl_sme2_mopa,
            },
    },
};

const std::array gemv_variants = {
    MatMulVariant{
        .name = "matmul_qai8_qai8_qsi8cxp",
        .acc_pack{
            .m = 1,
            .n = 2 * get_sme_vector_length<int32_t>(),
            .k = sizeof(int32_t) / sizeof(int8_t),
        },
        .acc_step{
            .m = 1,
            .n = 16 * get_sme_vector_length<int32_t>(),
            .k = sizeof(int32_t) / sizeof(int8_t),
        },

        .is_supported = cpu_has_sme2,

        .lhs_pack = std::nullopt,
        .rhs_pack = rhs_pack,
        .matmul = MatMulKernel{
            .get_m_step = kai_get_m_step_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
            .get_n_step = kai_get_n_step_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
            .get_mr = []() -> size_t { return 1; },
            .get_nr = kai_get_nr_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
            .get_kr = kai_get_kr_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
            .get_sr = kai_get_sr_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
            .get_packed_lhs_offset = kai_get_lhs_offset_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
            .get_packed_rhs_offset = kai_get_rhs_packed_offset_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
            .get_dst_offset = kai_get_dst_offset_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
            .get_dst_size = kai_get_dst_size_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
            .matmul =
                [](size_t m, size_t n, size_t k, const void* lhs, const void* rhs, void* dst, size_t dst_stride_row,
                   size_t dst_stride_col, const kai_matmul_requantize32_params* quant_param) {
                    kai_run_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot(
                        m, n, k, lhs, rhs, dst, dst_stride_row, dst_stride_col, quant_param);
                },
        },
    },
};

constexpr uint64_t seed = 0;               ///< Random seed used for tests
constexpr float output_clamp_rate = 0.1F;  ///< Clamping range in ration of output

/// Value range
template <typename T>
struct Range {
    T min;
    T max;

    [[nodiscard]] T range() const {
        return max - min;
    }
};

/// Quantization parameters
struct Quant {
    float scale;
    int32_t zero_point;
};

/// Reference test data
struct TestReference {
    Range<int8_t> clamp;

    Quant qa_lhs;
    Quant qa_dst;

    Buffer lhs_qai8;
    Buffer lhs_qai8_scales;
    Buffer lhs_qai8_zero_points;

    Buffer rhs_qsi8;
    Buffer rhs_scales;

    Buffer bias_qsi32;

    Buffer dst_qsi8_clamped;

    Buffer packed_lhs;
    Buffer packed_rhs;
};

/// Make sure that interface matches
static const kai_matmul_clamp_qai8_qai8p_qsi8cxp_ukernel matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot_interface
    [[maybe_unused]] = {
        .get_m_step = kai_get_m_step_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
        .get_n_step = kai_get_n_step_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
        .get_nr = kai_get_nr_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
        .get_kr = kai_get_kr_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
        .get_sr = kai_get_sr_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
        .get_lhs_offset = kai_get_lhs_offset_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
        .get_rhs_packed_offset = kai_get_rhs_packed_offset_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
        .get_dst_offset = kai_get_dst_offset_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
        .get_dst_size = kai_get_dst_size_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
        .run_matmul = kai_run_matmul_clamp_qai8_qai8_qsi8cxp2vlx4sb_1x16vl_sme2_dot,
};

/// Generate test reference data
static TestReference get_test_reference(const MatMulShape& shape, const MatMulVariant& variant) {
    // ============================================================
    // Generates input and reference output data
    // ============================================================

    // Generates the input data in floating-point.
    const auto lhs_f32 = fill_random<float>(shape.m * shape.k, seed);
    const auto rhs_f32 = fill_random<float>(shape.k * shape.n, seed);
    const auto bias_f32 = fill_random<float>(shape.n, seed);

    // Quantizes the input data.
    //   * LHS: 8-bit asymmetric per-matrix quantization.
    //   * RHS: 8-bit symmetric per-channel quantization.
    //   * Bias: 32-bit symmetric per-channel quantization.
    //
    //   Treat entire LHS as one row vector, to calculate one single pair of <scale, zero_point>
    auto [lhs_qai8, lhs_qai8_scales, lhs_qai8_zero_points] =
        quantize_asymmetric_per_block_dynamic<float, int8_t, float, int32_t>(
            lhs_f32.data(), 1, shape.m * shape.k, shape.m * shape.k);
    const auto lhs_scale = read_array<float>(lhs_qai8_scales.data(), 0);
    const auto lhs_zero_point = read_array<int32_t>(lhs_qai8_zero_points.data(), 0);

    // Transpose, then quantize symmetrically, then transpose back. This will give one
    // quantization value for each column
    const auto rhs_f32_t = transpose<float>(rhs_f32.data(), shape.k, shape.n);
    auto [rhs_qsi8_t, rhs_scales] =
        quantize_symmetric_per_block_dynamic<float, int8_t, float>(rhs_f32_t.data(), shape.n, shape.k, shape.k);
    auto rhs_qsi8 = transpose<int8_t>(rhs_qsi8_t.data(), shape.n, shape.k);

    // Multiply all bias values with the LHS scale
    const auto bias_scales = mul<float>(&lhs_scale, 1, 1, rhs_scales.data(), 1, shape.n);
    // Calculate quantized bias values, by treating bias as column, and
    // scale using RHS scales. This will scale each bias value indiviually
    auto bias_qsi32 =
        quantize_symmetric_per_block<float, int32_t, float>(bias_f32.data(), bias_scales.data(), shape.n, 1, 1);

    // Runs the reference implementation of matmul to produce floating-point result.
    const auto ref_dst_f32 =
        matmul_nt_t_quantized<int8_t, float, int32_t, int8_t, float, int32_t, int32_t, float, int32_t, float>(
            shape.m, shape.n, shape.k,                       // matmul shape
            lhs_qai8.data(), &lhs_scale, &lhs_zero_point,    // LHS, scaling factor and zero point
            shape.m, shape.k,                                // LHS quantization window shape
            rhs_qsi8_t.data(), rhs_scales.data(), nullptr,   // RHS scaling factors
            1, shape.k,                                      // RHS quantization window shape
            bias_qsi32.data(), bias_scales.data(), nullptr,  // Bias, scaling and zero points
            1                                                // Bias quantization window shape
        );

    // Computes the output quantization information and clamping limits.
    //
    // To get a realistic value for the output quantization information and clamping limits
    // and avoid uncontrolled saturation problem, these information will be calculated
    // based on the reference floating-point output.
    //
    // The clamping limits will be slightly narrower than the actual range of the output
    // so that a portion of the output will be clampped.
    const auto [dst_scales, dst_zero_points] =
        compute_asymmetric_per_block_quantization_info<float, int8_t, float, int32_t>(
            ref_dst_f32.data(), 1, shape.m * shape.n, shape.m * shape.n);
    const auto dst_scale = read_array<float>(dst_scales.data(), 0);
    const auto dst_zero_point = read_array<int32_t>(dst_zero_points.data(), 0);

    const auto ref_dst_f32_min = reduce_min<float>(ref_dst_f32.data(), shape.m * shape.n);
    const auto ref_dst_f32_max = reduce_max<float>(ref_dst_f32.data(), shape.m * shape.n);
    const auto ref_dst_f32_range = ref_dst_f32_max - ref_dst_f32_min;

    const auto ref_dst_f32_clamp_min = ref_dst_f32_min + ref_dst_f32_range * output_clamp_rate / 2;
    const auto ref_dst_f32_clamp_max = ref_dst_f32_max - ref_dst_f32_range * output_clamp_rate / 2;
    const auto dst_qai8_clamp_min =
        quantize_asymmetric<float, int8_t, int32_t>(ref_dst_f32_clamp_min, dst_scale, dst_zero_point);
    const auto dst_qai8_clamp_max =
        quantize_asymmetric<float, int8_t, int32_t>(ref_dst_f32_clamp_max, dst_scale, dst_zero_point);

    // Clamps and quantizes the reference output matrix.
    const auto ref_dst_f32_clamped =
        clamp<float>(ref_dst_f32.data(), shape.m * shape.n, ref_dst_f32_clamp_min, ref_dst_f32_clamp_max);
    auto ref_dst_qsi8_clamped = quantize_asymmetric_per_block<float, int8_t, float, int32_t>(
        ref_dst_f32_clamped.data(), &dst_scale, &dst_zero_point,  // values, scales, zero point
        1, shape.m * shape.n,                                     // data shape
        shape.m * shape.n                                         // quantization window width
    );

    // Runs the reference implementation of the packing functions.
    //
    // The reference packing functions cannot be executed earlier
    // because we need the reference floating-point output first to have
    // the quantization information.
    auto packed_lhs = reorder_block<int8_t>(lhs_qai8.data(), shape.m, shape.k, variant.acc_pack.m, variant.acc_pack.k);
    auto packed_rhs = matmul_pack_rhs_nxk_static_quantized<int8_t, float, int32_t>(
        rhs_qsi8_t.data(), rhs_scales.data(), lhs_scale, dst_scale, bias_qsi32.data(), lhs_zero_point, shape.n, shape.k,
        variant.acc_pack.n, variant.acc_pack.k);

    return {
        .clamp = {.min = dst_qai8_clamp_min, .max = dst_qai8_clamp_max},

        .qa_lhs = {.scale = lhs_scale, .zero_point = lhs_zero_point},
        .qa_dst = {.scale = dst_scale, .zero_point = dst_zero_point},

        .lhs_qai8 = std::move(lhs_qai8),
        .lhs_qai8_scales = std::move(lhs_qai8_scales),
        .lhs_qai8_zero_points = std::move(lhs_qai8_zero_points),

        .rhs_qsi8 = std::move(rhs_qsi8),
        .rhs_scales = std::move(rhs_scales),

        .bias_qsi32 = std::move(bias_qsi32),

        .dst_qsi8_clamped = std::move(ref_dst_qsi8_clamped),

        .packed_lhs = std::move(packed_lhs),
        .packed_rhs = std::move(packed_rhs),
    };
}

/// Test LHS packing
static void test_lhs_pack(
    const MatMulShape& shape, const MatMulVariant& variant, const Rect& output_area, const TestReference& reference) {
    KAI_ASSUME(variant.lhs_pack.has_value());

    const auto imp_packed_lhs_size =
        variant.lhs_pack->get_packed_lhs_size(shape.m, shape.k, variant.acc_pack.m, variant.acc_pack.k, 1);
    ASSERT_EQ(imp_packed_lhs_size, reference.packed_lhs.size());

    Buffer imp_packed_lhs(imp_packed_lhs_size);
    const auto imp_lhs_offset = variant.lhs_pack->get_lhs_offset(output_area.start_row(), shape.k * sizeof(int8_t));
    const auto imp_packed_lhs_offset = variant.lhs_pack->get_packed_lhs_offset(
        output_area.start_row(), shape.k, variant.acc_pack.m, variant.acc_pack.k, 1);

    variant.lhs_pack->pack(
        output_area.height(), shape.k, variant.acc_pack.m, variant.acc_pack.k, 1, 0,
        reference.lhs_qai8.data() + imp_lhs_offset, shape.k * sizeof(int8_t),
        imp_packed_lhs.data() + imp_packed_lhs_offset);

    const auto imp_packed_lhs_end_offset = output_area.end_row() < shape.m
        ? variant.lhs_pack->get_packed_lhs_offset(
              output_area.end_row(), shape.k, variant.acc_pack.m, variant.acc_pack.k, 1)
        : imp_packed_lhs_size;

    for (size_t i = 0; i < reference.packed_lhs.size(); ++i) {
        if (i >= imp_packed_lhs_offset && i < imp_packed_lhs_end_offset) {
            ASSERT_EQ(imp_packed_lhs[i], reference.packed_lhs[i]);
        } else {
            ASSERT_EQ(imp_packed_lhs[i], 0);
        }
    }
}

/// Test RHS packing
static void test_rhs_pack(
    const MatMulShape& shape, const MatMulVariant& variant, const Rect& output_area, const TestReference& reference) {
    const auto imp_packed_rhs_size = variant.rhs_pack.get_packed_rhs_size(shape.n, shape.k);
    ASSERT_EQ(imp_packed_rhs_size, reference.packed_rhs.size());
    Buffer imp_packed_rhs(imp_packed_rhs_size);

    const auto imp_rhs_offset = variant.rhs_pack.get_rhs_offset(output_area.start_col());
    const auto imp_bias_offset = variant.rhs_pack.get_bias_offset(output_area.start_col());
    const auto imp_scale_offset = variant.rhs_pack.get_scale_offset(output_area.start_col());
    const auto imp_packed_rhs_offset = variant.rhs_pack.get_packed_rhs_offset(output_area.start_col(), shape.k);

    const kai_rhs_pack_qsi8cx_params imp_pack_rhs_params{
        .lhs_zero_point = reference.qa_lhs.zero_point,
        .scale_multiplier = reference.qa_lhs.scale / reference.qa_dst.scale,
    };

    variant.rhs_pack.pack(
        1, output_area.width(), shape.k, variant.acc_pack.n, variant.acc_pack.k, 1, shape.n * sizeof(int8_t),
        reference.rhs_qsi8.data() + imp_rhs_offset, reference.bias_qsi32.data() + imp_bias_offset,
        reference.rhs_scales.data() + imp_scale_offset, imp_packed_rhs.data() + imp_packed_rhs_offset, 0,
        &imp_pack_rhs_params);

    const auto imp_packed_rhs_end_offset = output_area.end_col() < shape.n
        ? variant.rhs_pack.get_packed_rhs_offset(output_area.end_col(), shape.k)
        : imp_packed_rhs_size;

    size_t mismatches = 0;
    for (size_t i = 0; i < reference.packed_rhs.size(); ++i) {
        if (i >= imp_packed_rhs_offset && i < imp_packed_rhs_end_offset) {
            if (imp_packed_rhs[i] != reference.packed_rhs[i]) {
                mismatches += 1;
            }
        } else {
            if (imp_packed_rhs[i] != 0) {
                mismatches += 1;
            }
        }
    }
    ASSERT_EQ(mismatches, 0) << "There are an unexpected amount of mismatches in RHS packing";
}

/// Test MatMul of GEMM/GEMV like kernel
static void test_matmul(
    const MatMulShape& shape, const MatMulVariant& variant, const Rect& output_area, const TestReference& reference) {
    const auto imp_dst_size = variant.matmul.get_dst_size(shape.m, shape.n);
    ASSERT_EQ(imp_dst_size, reference.dst_qsi8_clamped.size());

    Buffer imp_dst(imp_dst_size);
    const auto [imp_lhs_offset, lhs_data] = [&]() -> std::tuple<size_t, const Buffer&> {
        if (variant.lhs_pack.has_value()) {
            return {variant.matmul.get_packed_lhs_offset(output_area.start_row(), shape.k), reference.packed_lhs};
        }
        return {output_area.start_row() * shape.k, reference.lhs_qai8};
    }();
    const size_t imp_packed_rhs_offset = variant.matmul.get_packed_rhs_offset(output_area.start_col(), shape.k);
    const size_t imp_dst_offset =
        variant.matmul.get_dst_offset(output_area.start_row(), output_area.start_col(), shape.n * sizeof(int8_t));
    ASSERT_EQ(imp_dst_offset, output_area.start_row() * shape.n + output_area.start_col());

    const kai_matmul_requantize32_params imp_main_params{
        .min_value = reference.clamp.min,
        .max_value = reference.clamp.max,
        .output_zero_point = reference.qa_dst.zero_point,
    };

    variant.matmul.matmul(
        output_area.height(), output_area.width(), shape.k, lhs_data.data() + imp_lhs_offset,
        reference.packed_rhs.data() + imp_packed_rhs_offset, imp_dst.data() + imp_dst_offset, shape.n * sizeof(int8_t),
        sizeof(int8_t), &imp_main_params);

    size_t mismatches = 0;
    for (size_t y = 0; y < shape.m; ++y) {
        for (size_t x = 0; x < shape.n; ++x) {
            const auto i = y * shape.n + x;
            const auto in_area = y >= output_area.start_row() && y < output_area.end_row() &&
                x >= output_area.start_col() && x < output_area.end_col();

            const auto imp_value = read_array<int8_t>(imp_dst.data(), i);
            const auto ref_value = in_area ? read_array<int8_t>(reference.dst_qsi8_clamped.data(), i) : 0;
            const auto error = std::abs(imp_value - ref_value);
            const auto threshold = in_area ? 1 : 0;

            mismatches += static_cast<size_t>(error > threshold);
        }
    }
    ASSERT_EQ(mismatches, 0) << "There are mismatched between reference result actual result";
}

using ThisTest = testing::TestWithParam<std::tuple<MatMulVariant, MatMulShape, MatrixPortion>>;

static std::string test_description(
    const MatMulVariant& variant,  //
    const MatMulShape& shape,      //
    const MatrixPortion& portion) {
    std::stringstream sstream;
    sstream << "Method_" << variant.name << "__M_"                                   //
            << shape.m << "__N_" << shape.n << "__K_" << shape.k                     //
            << "__PortionStartRow_" << static_cast<int>(portion.start_row() * 1000)  //
            << "__PortionStartCol_" << static_cast<int>(portion.start_col() * 1000)  //
            << "__PortionHeight_" << static_cast<int>(portion.height() * 1000)       //
            << "__PortionWidth_" << static_cast<int>(portion.width() * 1000);
    return sstream.str();
};

TEST_P(ThisTest, EndToEnd) {
    const auto& [variant, shape, output_portion] = GetParam();

    if (!variant.is_supported()) {
        GTEST_SKIP() << "CPU features are not supported by current CPU";
    }

    TestReference reference = get_test_reference(shape, variant);

    // Check scheduling parameters
    const auto imp_mr = variant.matmul.get_mr();
    const auto imp_nr = variant.matmul.get_nr();
    const auto imp_kr = variant.matmul.get_kr();
    const auto imp_sr = variant.matmul.get_sr();

    ASSERT_EQ(imp_mr, variant.acc_pack.m);
    ASSERT_EQ(imp_nr, variant.acc_pack.n);
    ASSERT_EQ(imp_kr, variant.acc_pack.k);
    ASSERT_EQ(imp_sr, 1);

    // Check that stepping is a multiple of accumulation
    const auto imp_m_step = variant.matmul.get_m_step();
    const auto imp_n_step = variant.matmul.get_n_step();
    ASSERT_EQ(imp_m_step, variant.acc_step.m);
    ASSERT_EQ(imp_n_step, variant.acc_step.n);

    // Test kernels. Note that packing and actual stepping might not be the same
    const auto pack_portion = output_portion.compute_portion(shape.m, shape.n, variant.acc_pack.m, variant.acc_pack.n);
    const auto matmul_portion =
        output_portion.compute_portion(shape.m, shape.n, variant.acc_step.m, variant.acc_step.n);
    if (variant.lhs_pack.has_value()) {
        test_lhs_pack(shape, variant, pack_portion, reference);
    }
    test_rhs_pack(shape, variant, pack_portion, reference);
    test_matmul(shape, variant, matmul_portion, reference);
}

INSTANTIATE_TEST_SUITE_P(
    matmul_clamp_qai8_qai8p_qsi8cxp, ThisTest,
    testing::Combine(
        testing::ValuesIn(gemm_variants),
        testing::ValuesIn({
            // clang-format off
            MatMulShape{  1,   1,  1},
            MatMulShape{  1,  49, 21},
            MatMulShape{ 20,  30, 40},
            MatMulShape{ 23,   1, 43},
            MatMulShape{ 32,  14,  1},
            MatMulShape{ 64,  64,  4},
            MatMulShape{123,  85, 45},
            MatMulShape{130, 130,  6},
            // clang-format on
        }),
        testing::ValuesIn({
            // clang-format off
            MatrixPortion(   0,    0,    1,    1), // Full matrix.
            MatrixPortion(   0,    0, 0.25, 0.25), // Top-left corner.
            MatrixPortion(0.75, 0.75,    1,    1), // Bottom-right corner.
            // clang-format on
        })),
    [](const auto& info) -> std::string {
        return test_description(
            std::get<MatMulVariant>(info.param),  //
            std::get<MatMulShape>(info.param),    //
            std::get<MatrixPortion>(info.param));
    });

INSTANTIATE_TEST_SUITE_P(
    matmul_clamp_qai8_qai8_qsi8cxp, ThisTest,
    testing::Combine(
        testing::ValuesIn(gemv_variants),
        testing::ValuesIn({
            // clang-format off
            MatMulShape{1,    1,    1},
            MatMulShape{1,   16,    4},
            MatMulShape{1,   16,   16},
            MatMulShape{1,   17,    4},
            MatMulShape{1,   32,   32},
            MatMulShape{1,   33,  200},
            MatMulShape{1,   64,    4},
            MatMulShape{1,   65,    4},
            MatMulShape{1,  300,   10},
            MatMulShape{1,  512,    4},
            MatMulShape{1, 1523,   10},
            // clang-format on
        }),
        testing::ValuesIn({
            // clang-format off
            MatrixPortion(0,   0, 1,  1), // Full matrix.
            MatrixPortion(0,  .5, 1, .5), // Right half
            MatrixPortion(0,   0, 1, .5), // Left half
            MatrixPortion(0, .25, 1, .5)  // Middle half
            // clang-format on
        })),
    [](const auto& info) -> std::string {
        return test_description(
            std::get<MatMulVariant>(info.param),  //
            std::get<MatMulShape>(info.param),    //
            std::get<MatrixPortion>(info.param));
    });
}  // namespace kai::test
