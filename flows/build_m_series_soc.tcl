# ==============================================================================
# VLSI SoC Studio — Apple M-Series Class SoC Tapeout Flow Script
# Target: Sub-3nm FinFET / GAAFET Hierarchical RTL-to-GDSII Signoff
# ==============================================================================

# ------------------------------------------------------------------------------
# Stage 1: Technology & PDK Collateral Ingestion
# ------------------------------------------------------------------------------
puts "=== [Stage 1/6] Ingesting ASAP7 FinFET PDK & Technology Libraries ==="
read_lef pdks/asap7/asap7_tech_1x.lef
read_lef pdks/asap7/asap7sc7p5t_28_L_1x.lef
read_lib pdks/asap7/asap7sc7p5t_LVT_FF_ccs.lib
read_lib pdks/asap7/asap7sc7p5t_SLVT_SS_ccs.lib
read_verilog designs/compute_cluster/core_tile.v
read_verilog designs/compute_cluster/gpu_slice.v
init_synthetic_soc
report_design

# ------------------------------------------------------------------------------
# Stage 2: Hierarchical Macro Floorplanning & UPF Multi-Voltage Intent
# ------------------------------------------------------------------------------
puts "=== [Stage 2/6] Hierarchical Floorplanning & UPF Multi-Voltage Intent ==="
read_upf flows/apple_dvfs_domains.upf
run_floorplan
write_abstract abstracts/pcore_0.lef
write_abstract abstracts/slc_0.lef

# ------------------------------------------------------------------------------
# Stage 3: Analytical Global Placement & FinFET Detailed Legalization
# ------------------------------------------------------------------------------
puts "=== [Stage 3/6] Analytical Global Placement (WA-HPWL + FFT) & Row Snapping ==="
run_placement
run_legalization
run_upf

# ------------------------------------------------------------------------------
# Stage 4: Zero-Skew Clock Tree Synthesis (CTS) & POCV Static Timing (STA)
# ------------------------------------------------------------------------------
puts "=== [Stage 4/6] Zero-Skew CTS (DME & H-Tree) & Multi-Corner STA (POCV) ==="
run_cts
run_sta
report_timing reports/sta_timing.rpt

# ------------------------------------------------------------------------------
# Stage 5: Mesh NoC Synthesis, 3D-IC UltraFusion, Global & Detailed Routing
# ------------------------------------------------------------------------------
puts "=== [Stage 5/6] 4.0 GHz Mesh NoC, UltraFusion Interposer, & 3D Routing ==="
run_noc
run_ultrafusion
run_global_route
run_detailed_route
run_thermal_ir

# ------------------------------------------------------------------------------
# Stage 6: Physical Verification (DRC/LVS), RL PPA Signoff, & Final GDSII Stream-Out
# ------------------------------------------------------------------------------
puts "=== [Stage 6/6] Sweep-Line DRC, LVS Isomorphism, RL PPA, & Final GDSII ==="
run_drc
run_lvs
run_rl_ppa 5
export_gdsii out/m_series_soc_final.gds

puts "=== [SUCCESS] M-Series SoC Tapeout Script Completed Cleanly ==="
