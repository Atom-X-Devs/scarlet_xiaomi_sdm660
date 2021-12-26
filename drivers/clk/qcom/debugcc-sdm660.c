// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2021, The Linux Foundation. All rights reserved.
 */

#define pr_fmt(fmt) "clk: %s: " fmt, __func__

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>

#include "clk-debug.h"
#include "common.h"

static struct measure_clk_data debug_mux_priv = {
	.ctl_reg = 0x62004,
	.status_reg = 0x62008,
	.xo_div4_cbcr = 0x43008,
};

static const char *const gcc_debug_mux_parent_names[] = {
	"measure_only_snoc_clk",
	"measure_only_cnoc_clk",
	"measure_only_cnoc_periph_clk",
	"measure_only_bimc_clk",
	"measure_only_ce1_clk",
	"measure_only_ipa_clk",
	"gcc_aggre2_ufs_axi_clk",
	"gcc_aggre2_usb3_axi_clk",
	"gcc_bimc_gfx_clk",
	"gcc_bimc_hmss_axi_clk",
	"gcc_bimc_mss_q6_axi_clk",
	"gcc_blsp1_ahb_clk",
	"gcc_blsp1_qup1_i2c_apps_clk",
	"gcc_blsp1_qup1_spi_apps_clk",
	"gcc_blsp1_qup2_i2c_apps_clk",
	"gcc_blsp1_qup2_spi_apps_clk",
	"gcc_blsp1_qup3_i2c_apps_clk",
	"gcc_blsp1_qup3_spi_apps_clk",
	"gcc_blsp1_qup4_i2c_apps_clk",
	"gcc_blsp1_qup4_spi_apps_clk",
	"gcc_blsp1_uart1_apps_clk",
	"gcc_blsp1_uart2_apps_clk",
	"gcc_blsp2_ahb_clk",
	"gcc_blsp2_qup1_i2c_apps_clk",
	"gcc_blsp2_qup1_spi_apps_clk",
	"gcc_blsp2_qup2_i2c_apps_clk",
	"gcc_blsp2_qup2_spi_apps_clk",
	"gcc_blsp2_qup3_i2c_apps_clk",
	"gcc_blsp2_qup3_spi_apps_clk",
	"gcc_blsp2_qup4_i2c_apps_clk",
	"gcc_blsp2_qup4_spi_apps_clk",
	"gcc_blsp2_uart1_apps_clk",
	"gcc_blsp2_uart2_apps_clk",
	"gcc_boot_rom_ahb_clk",
	"gcc_ce1_ahb_m_clk",
	"gcc_ce1_axi_m_clk",
	"gcc_cfg_noc_usb2_axi_clk",
	"gcc_cfg_noc_usb3_axi_clk",
	"gcc_dcc_ahb_clk",
	"gcc_gp1_clk",
	"gcc_gp2_clk",
	"gcc_gp3_clk",
	"gcc_gpu_bimc_gfx_clk",
	"gcc_gpu_cfg_ahb_clk",
	"gcc_hmss_dvm_bus_clk",
	"gcc_hmss_rbcpr_clk",
	"gcc_mmss_noc_cfg_ahb_clk",
	"gcc_mmss_sys_noc_axi_clk",
	"gcc_mss_cfg_ahb_clk",
	"gcc_mss_mnoc_bimc_axi_clk",
	"gcc_mss_q6_bimc_axi_clk",
	"gcc_mss_snoc_axi_clk",
	"gcc_pdm2_clk",
	"gcc_pdm_ahb_clk",
	"gcc_prng_ahb_clk",
	"gcc_qspi_ahb_clk",
	"gcc_qspi_ser_clk",
	"gcc_sdcc1_ahb_clk",
	"gcc_sdcc1_apps_clk",
	"gcc_sdcc1_ice_core_clk",
	"gcc_sdcc2_ahb_clk",
	"gcc_sdcc2_apps_clk",
	"gcc_ufs_ahb_clk",
	"gcc_ufs_axi_clk",
	"gcc_ufs_ice_core_clk",
	"gcc_ufs_phy_aux_clk",
	"gcc_ufs_rx_symbol_0_clk",
	"gcc_ufs_rx_symbol_1_clk",
	"gcc_ufs_tx_symbol_0_clk",
	"gcc_ufs_unipro_core_clk",
	"gcc_usb20_master_clk",
	"gcc_usb20_mock_utmi_clk",
	"gcc_usb20_sleep_clk",
	"gcc_usb30_master_clk",
	"gcc_usb30_mock_utmi_clk",
	"gcc_usb30_sleep_clk",
	"gcc_usb3_phy_aux_clk",
	"gcc_usb3_phy_pipe_clk",
	"gcc_usb_phy_cfg_ahb2phy_clk",
	"gpu_cc_debug_mux",
	"mmss_cc_debug_mux",
	"cpu_cc_debug_mux",
};

static int gcc_debug_mux_sels[] = {
	0x0,		/* measure_only_snoc_clk */
	0xE,		/* measure_only_cnoc_clk */
	0x198,		/* measure_only_cnoc_periph_clk */
	0x19D,		/* measure_only_bimc_clk */
	0x97,		/* measure_only_ce1_clk */
	0x11b,		/* measure_only_ipa_clk */
	0x10B,		/* gcc_aggre2_ufs_axi_clk */
	0x10A,		/* gcc_aggre2_usb3_axi_clk */
	0xAC,		/* gcc_bimc_gfx_clk */
	0xBB,		/* gcc_bimc_hmss_axi_clk */
	0xA3,		/* gcc_bimc_mss_q6_axi_clk */
	0x4A,		/* gcc_blsp1_ahb_clk */
	0x4D,		/* gcc_blsp1_qup1_i2c_apps_clk */
	0x4C,		/* gcc_blsp1_qup1_spi_apps_clk */
	0x51,		/* gcc_blsp1_qup2_i2c_apps_clk */
	0x50,		/* gcc_blsp1_qup2_spi_apps_clk */
	0x55,		/* gcc_blsp1_qup3_i2c_apps_clk */
	0x54,		/* gcc_blsp1_qup3_spi_apps_clk */
	0x59,		/* gcc_blsp1_qup4_i2c_apps_clk */
	0x58,		/* gcc_blsp1_qup4_spi_apps_clk */
	0x4E,		/* gcc_blsp1_uart1_apps_clk */
	0x52,		/* gcc_blsp1_uart2_apps_clk */
	0x5E,		/* gcc_blsp2_ahb_clk */
	0x61,		/* gcc_blsp2_qup1_i2c_apps_clk */
	0x60,		/* gcc_blsp2_qup1_spi_apps_clk */
	0x65,		/* gcc_blsp2_qup2_i2c_apps_clk */
	0x64,		/* gcc_blsp2_qup2_spi_apps_clk */
	0x69,		/* gcc_blsp2_qup3_i2c_apps_clk */
	0x68,		/* gcc_blsp2_qup3_spi_apps_clk */
	0x6D,		/* gcc_blsp2_qup4_i2c_apps_clk */
	0x6C,		/* gcc_blsp2_qup4_spi_apps_clk */
	0x62,		/* gcc_blsp2_uart1_apps_clk */
	0x66,		/* gcc_blsp2_uart2_apps_clk */
	0x7A,		/* gcc_boot_rom_ahb_clk */
	0x99,		/* gcc_ce1_ahb_m_clk */
	0x98,		/* gcc_ce1_axi_m_clk */
	0x168,		/* gcc_cfg_noc_usb2_axi_clk */
	0x14,		/* gcc_cfg_noc_usb3_axi_clk */
	0x119,		/* gcc_dcc_ahb_clk */
	0xDF,		/* gcc_gp1_clk */
	0xE0,		/* gcc_gp2_clk */
	0xE1,		/* gcc_gp3_clk */
	0x13F,		/* gcc_gpu_bimc_gfx_clk */
	0x13B,		/* gcc_gpu_cfg_ahb_clk */
	0xBF,		/* gcc_hmss_dvm_bus_clk */
	0xBC,		/* gcc_hmss_rbcpr_clk */
	0x20,		/* gcc_mmss_noc_cfg_ahb_clk */
	0x1F,		/* gcc_mmss_sys_noc_axi_clk */
	0x11F,		/* gcc_mss_cfg_ahb_clk */
	0x120,		/* gcc_mss_mnoc_bimc_axi_clk */
	0x124,		/* gcc_mss_q6_bimc_axi_clk */
	0x123,		/* gcc_mss_snoc_axi_clk */
	0x74,		/* gcc_pdm2_clk */
	0x72,		/* gcc_pdm_ahb_clk */
	0x75,		/* gcc_prng_ahb_clk */
	0x172,		/* gcc_qspi_ahb_clk */
	0x173,		/* gcc_qspi_ser_clk */
	0x16E,		/* gcc_sdcc1_ahb_clk */
	0x16D,		/* gcc_sdcc1_apps_clk */
	0x16F,		/* gcc_sdcc1_ice_core_clk */
	0x47,		/* gcc_sdcc2_ahb_clk */
	0x46,		/* gcc_sdcc2_apps_clk */
	0xEB,		/* gcc_ufs_ahb_clk */
	0xEA,		/* gcc_ufs_axi_clk */
	0xF1,		/* gcc_ufs_ice_core_clk */
	0xF2,		/* gcc_ufs_phy_aux_clk */
	0xED,		/* gcc_ufs_rx_symbol_0_clk */
	0x162,		/* gcc_ufs_rx_symbol_1_clk */
	0xEC,		/* gcc_ufs_tx_symbol_0_clk */
	0xF0,		/* gcc_ufs_unipro_core_clk */
	0x169,		/* gcc_usb20_master_clk */
	0x16B,		/* gcc_usb20_mock_utmi_clk */
	0x16A,		/* gcc_usb20_sleep_clk */
	0x3C,		/* gcc_usb30_master_clk */
	0x3E,		/* gcc_usb30_mock_utmi_clk */
	0x3D,		/* gcc_usb30_sleep_clk */
	0x3F,		/* gcc_usb3_phy_aux_clk */
	0x40,		/* gcc_usb3_phy_pipe_clk */
	0x45,		/* gcc_usb_phy_cfg_ahb2phy_clk */
	0x13D,		/* gpu_cc_debug_mux */
	0x22,		/* mmss_cc_debug_mux */
	0xC0,		/* cpu_cc_debug_mux */
};

static struct clk_debug_mux gcc_debug_mux = {
	.priv = &debug_mux_priv,
	.en_mask = BIT(16),
	.debug_offset = 0x62000,
	.post_div_offset = 0x62000,
	.cbcr_offset = 0x62000,
	.src_sel_mask = 0x3FF,
	.src_sel_shift = 0,
	.post_div_mask = 0xF000,
	.post_div_shift = 12,
	.post_div_val = 1,
	.mux_sels = gcc_debug_mux_sels,
	.hw.init = &(struct clk_init_data){
		.name = "gcc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = gcc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(gcc_debug_mux_parent_names),
		.flags = CLK_IS_MEASURE,
	},
};

static const char *const gpu_cc_debug_mux_parent_names[] = {
	"gpucc_gfx3d_clk",
	"gpucc_rbbmtimer_clk",
	"gpucc_rbcpr_clk",
};

static int gpu_cc_debug_mux_sels[] = {
	0x8,		/* gpucc_gfx3d_clk */
	0x5,		/* gpucc_rbbmtimer_clk */
	0x3,		/* gpucc_rbcpr_clk */
};

static struct clk_debug_mux gpu_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.en_mask = BIT(16),
	.debug_offset = 0x120,
	.post_div_offset = 0x120,
	.cbcr_offset = 0x120,
	.src_sel_mask = 0xF,
	.src_sel_shift = 0,
	.post_div_mask = 0x60000,
	.post_div_shift = 17,
	.post_div_val = 3,
	.mux_sels = gpu_cc_debug_mux_sels,
	.hw.init = &(struct clk_init_data){
		.name = "gpu_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = gpu_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(gpu_cc_debug_mux_parent_names),
		.flags = CLK_IS_MEASURE,
	},
};

static const char *const cpu_cc_debug_mux_parent_names[] = {
	"measure_only_perfcl_clk",
	"measure_only_pwrcl_clk",
};

static int cpu_cc_debug_mux_sels[] = {
	0x1,		/* measure_only_perfcl_clk */
	0x0,		/* measure_only_pwrcl_clk */
};

static int apss_cc_debug_mux_pre_divs[] = {
	0x8,		/* measure_only_perfcl_clk */
	0x8,		/* measure_only_pwrcl_clk */
};

static struct clk_debug_mux cpu_cc_debug_mux = {
	.priv = &debug_mux_priv,cuba
	.debug_offset = 0x0,
	.post_div_offset = 0x0,
	.cbcr_offset = U32_MAX,
	.src_sel_mask = 0x3FF00,
	.src_sel_shift = 8,
	.post_div_mask = 0xF0000000,
	.post_div_shift = 28,
	.post_div_val = 1,
	.mux_sels = cpu_cc_debug_mux_sels,
	.pre_div_vals = apss_cc_debug_mux_pre_divs,
	.hw.init = &(struct clk_init_data){
		.name = "cpu_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = cpu_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(cpu_cc_debug_mux_parent_names),
		.flags = CLK_IS_MEASURE,
	},
};

static const char *const mmss_cc_debug_mux_parent_names[] = {
	"measure_only_mmssnoc_axi_clk",
	"mmss_bimc_smmu_ahb_clk",
	"mmss_bimc_smmu_axi_clk",
	"mmss_camss_ahb_clk",
	"mmss_camss_cci_ahb_clk",
	"mmss_camss_cci_clk",
	"mmss_camss_cphy_csid0_clk",
	"mmss_camss_cphy_csid1_clk",
	"mmss_camss_cphy_csid2_clk",
	"mmss_camss_cphy_csid3_clk",
	"mmss_camss_cpp_ahb_clk",
	"mmss_camss_cpp_axi_clk",
	"mmss_camss_cpp_clk",
	"mmss_camss_cpp_vbif_ahb_clk",
	"mmss_camss_csi0_ahb_clk",
	"mmss_camss_csi0_clk",
	"mmss_camss_csi0phytimer_clk",
	"mmss_camss_csi0pix_clk",
	"mmss_camss_csi0rdi_clk",
	"mmss_camss_csi1_ahb_clk",
	"mmss_camss_csi1_clk",
	"mmss_camss_csi1phytimer_clk",
	"mmss_camss_csi1pix_clk",
	"mmss_camss_csi1rdi_clk",
	"mmss_camss_csi2_ahb_clk",
	"mmss_camss_csi2_clk",
	"mmss_camss_csi2phytimer_clk",
	"mmss_camss_csi2pix_clk",
	"mmss_camss_csi2rdi_clk",
	"mmss_camss_csi3_ahb_clk",
	"mmss_camss_csi3_clk",
	"mmss_camss_csi3pix_clk",
	"mmss_camss_csi3rdi_clk",
	"mmss_camss_csi_vfe0_clk",
	"mmss_camss_csi_vfe1_clk",
	"mmss_camss_csiphy0_clk",
	"mmss_camss_csiphy1_clk",
	"mmss_camss_csiphy2_clk",
	"mmss_camss_gp0_clk",
	"mmss_camss_gp1_clk",
	"mmss_camss_ispif_ahb_clk",
	"mmss_camss_jpeg0_clk",
	"mmss_camss_jpeg_ahb_clk",
	"mmss_camss_jpeg_axi_clk",
	"mmss_camss_mclk0_clk",
	"mmss_camss_mclk1_clk",
	"mmss_camss_mclk2_clk",
	"mmss_camss_mclk3_clk",
	"mmss_camss_micro_ahb_clk",
	"mmss_camss_top_ahb_clk",
	"mmss_camss_vfe0_ahb_clk",
	"mmss_camss_vfe0_clk",
	"mmss_camss_vfe0_stream_clk",
	"mmss_camss_vfe1_ahb_clk",
	"mmss_camss_vfe1_clk",
	"mmss_camss_vfe1_stream_clk",
	"mmss_camss_vfe_vbif_ahb_clk",
	"mmss_camss_vfe_vbif_axi_clk",
	"mmss_csiphy_ahb2crif_clk",
	"mmss_mdss_ahb_clk",
	"mmss_mdss_axi_clk",
	"mmss_mdss_byte0_clk",
	"mmss_mdss_byte0_intf_clk",
	"mmss_mdss_byte1_clk",
	"mmss_mdss_byte1_intf_clk",
	"mmss_mdss_dp_aux_clk",
	"mmss_mdss_dp_crypto_clk",
	"mmss_mdss_dp_gtc_clk",
	"mmss_mdss_dp_link_clk",
	"mmss_mdss_dp_link_intf_clk",
	"mmss_mdss_dp_pixel_clk",
	"mmss_mdss_esc0_clk",
	"mmss_mdss_esc1_clk",
	"mmss_mdss_hdmi_dp_ahb_clk",
	"mmss_mdss_mdp_clk",
	"mmss_mdss_pclk0_clk",
	"mmss_mdss_pclk1_clk",
	"mmss_mdss_rot_clk",
	"mmss_mdss_vsync_clk",
	"mmss_misc_ahb_clk",
	"mmss_misc_cxo_clk",
	"mmss_mnoc_ahb_clk",
	"mmss_snoc_dvm_axi_clk",
	"mmss_throttle_camss_axi_clk",
	"mmss_throttle_mdss_axi_clk",
	"mmss_throttle_video_axi_clk",
	"mmss_video_ahb_clk",
	"mmss_video_axi_clk",
	"mmss_video_core_clk",
	"mmss_video_subcore0_clk",
};

static int mmss_cc_debug_mux_sels[] = {
	0x4,		/* measure_only_mmssnoc_axi_clk" */
	0xC,		/* mmss_bimc_smmu_ahb_clk */
	0xD,		/* mmss_bimc_smmu_axi_clk */
	0x37,		/* mmss_camss_ahb_clk" */
	0x2E,		/* mmss_camss_cci_ahb_clk */
	0x2D,		/* mmss_camss_cci_clk" */
	0x8D,		/* mmss_camss_cphy_csid0_clk */
	0x8E,		/* mmss_camss_cphy_csid1_clk */
	0x8F,		/* mmss_camss_cphy_csid2_clk */
	0x90,		/* mmss_camss_cphy_csid3_clk */
	0x3B,		/* mmss_camss_cpp_ahb_clk */
	0x7A,		/* mmss_camss_cpp_axi_clk */
	0x3A,		/* mmss_camss_cpp_clk" */
	0x73,		/* mmss_camss_cpp_vbif_ahb_clk */
	0x42,		/* mmss_camss_csi0_ahb_clk */
	0x41,		/* mmss_camss_csi0_clk */
	0x2F,		/* mmss_camss_csi0phytimer_clk */
	0x45,		/* mmss_camss_csi0pix_clk */
	0x44,		/* mmss_camss_csi0rdi_clk */
	0x47,		/* mmss_camss_csi1_ahb_clk */
	0x46,		/* mmss_camss_csi1_clk */
	0x30,		/* mmss_camss_csi1phytimer_clk */
	0x4A,		/* mmss_camss_csi1pix_clk */
	0x49,		/* mmss_camss_csi1rdi_clk */
	0x4C,		/* mmss_camss_csi2_ahb_clk */
	0x4B,		/* mmss_camss_csi2_clk */
	0x31,		/* mmss_camss_csi2phytimer_clk */
	0x4F,		/* mmss_camss_csi2pix_clk */
	0x4E,		/* mmss_camss_csi2rdi_clk */
	0x51,		/* mmss_camss_csi3_ahb_clk */
	0x50,		/* mmss_camss_csi3_clk */
	0x54,		/* mmss_camss_csi3pix_clk" */
	0x53,		/* mmss_camss_csi3rdi_clk */
	0x3F,		/* mmss_camss_csi_vfe0_clk */
	0x40,		/* mmss_camss_csi_vfe1_clk */
	0x43,		/* mmss_camss_csiphy0_clk */
	0x85,		/* mmss_camss_csiphy1_clk */
	0x88,		/* mmss_camss_csiphy2_clk */
	0x27,		/* mmss_camss_gp0_clk" */
	0x28,		/* mmss_camss_gp1_clk" */
	0x33,		/* mmss_camss_ispif_ahb_clk */
	0x32,		/* mmss_camss_jpeg0_clk */
	0x35,		/* mmss_camss_jpeg_ahb_clk */
	0x36,		/* mmss_camss_jpeg_axi_clk */
	0x29,		/* mmss_camss_mclk0_clk */
	0x2A,		/* mmss_camss_mclk1_clk */
	0x2B,		/* mmss_camss_mclk2_clk */
	0x2C,		/* mmss_camss_mclk3_clk */
	0x26,		/* mmss_camss_micro_ahb_clk */
	0x25,		/* mmss_camss_top_ahb_clk */
	0x86,		/* mmss_camss_vfe0_ahb_clk */
	0x38,		/* mmss_camss_vfe0_clk */
	0x71,		/* mmss_camss_vfe0_stream_clk */
	0x87,		/* mmss_camss_vfe1_ahb_clk */
	0x39,		/* mmss_camss_vfe1_clk */
	0x72,		/* mmss_camss_vfe1_stream_clk */
	0x3C,		/* mmss_camss_vfe_vbif_ahb_clk */
	0x3D,		/* mmss_camss_vfe_vbif_axi_clk */
	0xB8,		/* mmss_csiphy_ahb2crif_clk */
	0x22,		/* mmss_mdss_ahb_clk" */
	0x24,		/* mmss_mdss_axi_clk" */
	0x1E,		/* mmss_mdss_byte0_clk */
	0xAD,		/* mmss_mdss_byte0_intf_clk */
	0x1F,		/* mmss_mdss_byte1_clk */
	0xB6,		/* mmss_mdss_byte1_intf_clk */
	0x9C,		/* mmss_mdss_dp_aux_clk */
	0x9A,		/* mmss_mdss_dp_crypto_clk */
	0x9D,		/* mmss_mdss_dp_gtc_clk */
	0x98,		/* mmss_mdss_dp_link_clk */
	0x99,		/* mmss_mdss_dp_link_intf_clk */
	0x9B,		/* mmss_mdss_dp_pixel_clk */
	0x20,		/* mmss_mdss_esc0_clk" */
	0x21,		/* mmss_mdss_esc1_clk" */
	0x23,		/* mmss_mdss_hdmi_dp_ahb_clk */
	0x14,		/* mmss_mdss_mdp_clk" */
	0x16,		/* mmss_mdss_pclk0_clk */
	0x17,		/* mmss_mdss_pclk1_clk */
	0x12,		/* mmss_mdss_rot_clk" */
	0x1C,		/* mmss_mdss_vsync_clk */
	0x3,		/* mmss_misc_ahb_clk" */
	0x77,		/* mmss_misc_cxo_clk" */
	0x1,		/* mmss_mnoc_ahb_clk" */
	0x13,		/* mmss_snoc_dvm_axi_clk */
	0xAA,		/* mmss_throttle_camss_axi_clk */
	0xAB,		/* mmss_throttle_mdss_axi_clk */
	0xAC,		/* mmss_throttle_video_axi_clk */
	0x11,		/* mmss_video_ahb_clk" */
	0xF,		/* mmss_video_axi_clk" */
	0xE,		/* mmss_video_core_clk */
	0x1A,		/* mmss_video_subcore0_clk */
};

static struct clk_debug_mux mmss_cc_debug_mux = {
	.priv = &debug_mux_priv,
	.en_mask = BIT(12),
	.debug_offset = 0x900,
	.post_div_offset = 0x900,
	.cbcr_offset = 0x900,
	.src_sel_mask = 0x1FF,
	.src_sel_shift = 0,
	.post_div_mask = 0x6000,
	.post_div_shift = 13,
	.post_div_val = 3,
	.mux_sels = mmss_cc_debug_mux_sels,
	.hw.init = &(struct clk_init_data){
		.name = "mmss_cc_debug_mux",
		.ops = &clk_debug_mux_ops,
		.parent_names = mmss_cc_debug_mux_parent_names,
		.num_parents = ARRAY_SIZE(mmss_cc_debug_mux_parent_names),
		.flags = CLK_IS_MEASURE,
	},
};

static struct mux_regmap_names mux_list[] = {
	{ .mux = &gcc_debug_mux, .regmap_name = "qcom,gcc" },
	{ .mux = &gpu_cc_debug_mux, .regmap_name = "qcom,gpu" },
	{ .mux = &cpu_cc_debug_mux, .regmap_name = "qcom,cpu" },
	{ .mux = &mmss_cc_debug_mux, .regmap_name = "qcom,mmss" },
};

static struct clk_dummy measure_only_perfcl_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_perfcl_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_pwrcl_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_pwrcl_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_snoc_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_snoc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_cnoc_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_cnoc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_cnoc_periph_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_cnoc_periph_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_bimc_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_bimc_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_ce1_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_ce1_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_ipa_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_ipa_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_dummy measure_only_mmssnoc_axi_clk = {
	.rrate = 1000,
	.hw.init = &(struct clk_init_data){
		.name = "measure_only_mmssnoc_axi_clk",
		.ops = &clk_dummy_ops,
	},
};

static struct clk_hw *debugcc_sdm660_hws[] = {
	&measure_only_snoc_clk.hw,
	&measure_only_cnoc_clk.hw,
	&measure_only_cnoc_periph_clk.hw,
	&measure_only_bimc_clk.hw,
	&measure_only_ce1_clk.hw,
	&measure_only_ipa_clk.hw,
	&measure_only_mmssnoc_axi_clk.hw,
	&measure_only_perfcl_clk.hw,
	&measure_only_pwrcl_clk.hw,
};

static const struct of_device_id clk_debug_match_table[] = {
	{ .compatible = "qcom,sdm660-debugcc" },
	{}
};

static int clk_debug_sdm660_probe(struct platform_device *pdev)
{
	struct clk *clk;
	int ret = 0, i;

	BUILD_BUG_ON(ARRAY_SIZE(gcc_debug_mux_parent_names) !=
		     ARRAY_SIZE(gcc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(gpu_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(gpu_cc_debug_mux_sels));
	BUILD_BUG_ON(ARRAY_SIZE(mmss_cc_debug_mux_parent_names) !=
		ARRAY_SIZE(mmss_cc_debug_mux_sels));

	clk = devm_clk_get(&pdev->dev, "xo_clk_src");
	if (IS_ERR(clk)) {
		if (PTR_ERR(clk) != -EPROBE_DEFER)
			dev_err(&pdev->dev, "Unable to get xo clock\n");
		return PTR_ERR(clk);
	}

	debug_mux_priv.cxo = clk;

	for (i = 0; i < ARRAY_SIZE(mux_list); i++) {
		ret = map_debug_bases(pdev, mux_list[i].regmap_name,
				      mux_list[i].mux);
		if (ret == -EBADR)
			continue;
		else if (ret)
			return ret;

		clk = devm_clk_register(&pdev->dev, &mux_list[i].mux->hw);
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "Unable to register %s, err:(%d)\n",
				mux_list[i].mux->hw.init->name, PTR_ERR(clk));
			return PTR_ERR(clk);
		}
	}

	for (i = 0; i < ARRAY_SIZE(debugcc_sdm660_hws); i++) {
		clk = devm_clk_register(&pdev->dev, debugcc_sdm660_hws[i]);
		if (IS_ERR(clk)) {
			dev_err(&pdev->dev, "Unable to register %s, err:(%d)\n",
			debugcc_sdm660_hws[i]->init->name, PTR_ERR(clk));
			return PTR_ERR(clk);
		}
	}

	ret = clk_debug_measure_register(&gcc_debug_mux.hw);
	if (ret)
		dev_err(&pdev->dev, "Could not register Measure clock\n");

	return ret;
}

static struct platform_driver clk_debug_driver = {
	.probe = clk_debug_sdm660_probe,
	.driver = {
		.name = "debugcc-sdm660",
		.of_match_table = clk_debug_match_table,
	},
};

static int __init clk_debug_sdm660_init(void)
{
	return platform_driver_register(&clk_debug_driver);
}
fs_initcall(clk_debug_sdm660_init);

MODULE_DESCRIPTION("Qualcomm SDM630/SDM660 DEBUGCC Driver");
MODULE_LICENSE("GPL v2");
