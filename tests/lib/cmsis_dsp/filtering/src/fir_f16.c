/*
 * Copyright (c) 2021 Stephanos Ioannidis <root@stephanos.io>
 * Copyright (C) 2010-2021 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/ztest.h>
#include <zephyr/kernel.h>
#include <stdlib.h>
#include <arm_math_f16.h>
#include "../../common/test_common.h"

#include "fir_f16.pat"

#define SNR_ERROR_THRESH	((float32_t)60)
#define REL_ERROR_THRESH	(1.0e-2)

#define COEFF_PADDING		(4)

static void test_arm_fir_f16(void)
{
	size_t sample_index, block_index;
	size_t block_size, tap_count;
	size_t sample_count = ARRAY_SIZE(in_config) / 2;
	size_t length = ARRAY_SIZE(ref_val);
	const uint16_t *config = in_config;
	const float16_t *input = (const float16_t *)in_val;
	const float16_t *coeff = (const float16_t *)in_coeff;
	const float16_t *ref = (const float16_t *)ref_val;
	float16_t *state, *output_buf, *output;
	arm_fir_instance_f16 inst;
#if defined(CONFIG_ARMV8_1_M_MVEF) && defined(CONFIG_FPU)
	float16_t coeff_padded[32];
	int round;
#endif

	/* Allocate buffers */
	state = malloc(2 * 47 * sizeof(float16_t));
	zassert_not_null(state, ASSERT_MSG_BUFFER_ALLOC_FAILED);

	output_buf = malloc(length * sizeof(float16_t));
	zassert_not_null(output_buf, ASSERT_MSG_BUFFER_ALLOC_FAILED);

	output = output_buf;

	/* Enumerate samples */
	for (sample_index = 0; sample_index < sample_count; sample_index++) {
		/* Resolve sample configurations */
		block_size = config[0];
		tap_count = config[1];

#if defined(CONFIG_ARMV8_1_M_MVEF) && defined(CONFIG_FPU)
		/* Copy coefficients and pad to zero */
		memset(coeff_padded, 127, sizeof(coeff_padded));
		round = tap_count / COEFF_PADDING;
		if ((round * COEFF_PADDING) < tap_count) {
			round++;
		}
		round = round * COEFF_PADDING;
		memset(coeff_padded, 0, round * sizeof(float16_t));
		memcpy(coeff_padded, coeff, tap_count * sizeof(float16_t));
#endif

		/* Initialise instance */
#if defined(CONFIG_ARMV8_1_M_MVEF) && defined(CONFIG_FPU)
		arm_fir_init_f16(&inst, tap_count, coeff_padded, state, block_size);
#else
		arm_fir_init_f16(&inst, tap_count, coeff, state, block_size);
#endif

		/* Reset input pointer */
		input = (const float16_t *)in_val;

		/* Enumerate blocks */
		for (block_index = 0; block_index < 2; block_index++) {
			/* Run test function */
			arm_fir_f16(&inst, input, output, block_size);

			/* Increment pointers */
			input += block_size;
			output += block_size;
		}

		/* Increment pointers */
		coeff += tap_count;
		config += 2;
	}

	/* Validate output */
	zassert_true(
		test_snr_error_f16(length, output_buf, ref, SNR_ERROR_THRESH),
		ASSERT_MSG_SNR_LIMIT_EXCEED);

	zassert_true(
		test_rel_error_f16(length, output_buf, ref, REL_ERROR_THRESH),
		ASSERT_MSG_REL_ERROR_LIMIT_EXCEED);

	/* Free buffers */
	free(state);
	free(output_buf);
}

void test_filtering_fir_f16(void)
{
	ztest_test_suite(filtering_fir_f16,
		ztest_unit_test(test_arm_fir_f16)
		);

	ztest_run_test_suite(filtering_fir_f16);
}
