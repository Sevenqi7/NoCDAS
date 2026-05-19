# RTL Verilator Router Setup

This note explains how to build and run the Verilated Router model.  There are
two supported modes:

- **Route-only RTL router**: default mode.  `ENABLE_CNOC_MFU=OFF`; the RTL model
  owns regular router datapath behavior, while cNoC/MFU storage and compute
  side effects remain in the C++ model.
- **RTL-owned cNoC/MFU router**: quantized cNoC mode.  `ENABLE_CNOC_MFU=ON`;
  the RTL model instantiates the MFU side pipeline and owns type4/type5
  storage/compute behavior.

`ROUTER_ENABLE_COSIM` is only the Verilator/C++ ABI and debug-status macro.  It
does not imply that cNoC/MFU is enabled.

## 1. Verilator

Use Verilator 5.048 or newer.  If your system `verilator --version` is missing
or too old, build it locally:

```bash
sudo apt-get update
sudo apt-get install -y git help2man perl python3 make g++ autoconf flex bison \
  libfl-dev libfl2 zlib1g-dev
git clone https://github.com/verilator/verilator.git
cd verilator
git checkout v5.048
autoconf
./configure --prefix="$HOME/local/verilator"
make -j"$(nproc)"
make install
```

Then put the new binary on `PATH`:

```bash
export PATH="$HOME/local/verilator/bin:$PATH"
export VERILATOR_ROOT="$HOME/local/verilator/share/verilator"
verilator --version
```

## 2. Build Modes

### Route-Only Router

This is the default and is the preferred build for analyzing the regular
VC/QoS/XY router without MFU/cNoC timing noise:

```bash
cmake -S . -B build_router_noc_only -DCNOC_QUANT_GOLDEN=OFF -DENABLE_CNOC_MFU=OFF
cmake --build build_router_noc_only -j"$(nproc)"
```

Running with `-rtl_router` in this mode prints:

```text
Verilated RTL Router backend enabled (cNoC/MFU handled by C++ model)
```

That means the Verilated RTL is used for router routing/VC/switch/crossbar
behavior, but type4/type5 cNoC side effects are still applied by the C++ model
when flits leave each router.

### RTL-Owned cNoC/MFU

Use this when comparing the quantized cNoC path:

```bash
cmake -S . -B build_router_cnoc_quant -DCNOC_QUANT_GOLDEN=ON -DENABLE_CNOC_MFU=ON
cmake --build build_router_cnoc_quant -j"$(nproc)"
```

`CNOC_QUANT_GOLDEN=ON` forces `ENABLE_CNOC_MFU=ON` in CMake.  Running with
`-rtl_router` in this mode prints:

```text
Verilated RTL Router backend enabled (RTL-owned cNoC/MFU path)
```

In this mode the RTL `Router` instantiates the cNoC/MFU side path, including
`mfu_arbiter`, `cnoc_mfu`, and the ALU wrapper path.

## 3. Manual Trace Runs

Generate a C++ reference trace:

```bash
./build_router_noc_only/NoCDASim -trace
cp trace.log trace_cpp.log
```

Generate a route-only RTL trace:

```bash
./build_router_noc_only/NoCDASim -rtl_router -trace
cp trace.log trace_rtl_route_only.log
```

Compare route/type behavior while ignoring cNoC numeric fields:

```bash
python3 script/compare_traces.py --ignore-cnoc-values \
  trace_cpp.log trace_rtl_route_only.log
```

For RTL-owned quantized cNoC:

```bash
./build_router_cnoc_quant/NoCDASim -trace
cp trace.log trace_cpp_quant.log
./build_router_cnoc_quant/NoCDASim -rtl_router -trace
cp trace.log trace_rtl_quant.log
python3 script/compare_traces.py --quant-cnoc trace_cpp_quant.log trace_rtl_quant.log
```

## 4. Regression Scripts

Preferred route-only smoke:

```bash
bash script/run_rtl_trace_regression.sh --compare route --build-dir build_router_noc_only
```

Expected key result:

```text
failure_category: none
path_diff_golden_minus_candidate: 0
path_diff_candidate_minus_golden: 0
```

Preferred quantized cNoC regression:

```bash
bash script/run_rtl_trace_regression.sh --quant-cnoc --build-dir build_router_cnoc_quant
```

Expected key result:

```text
failure_category: none
path_diff_golden_minus_candidate: 0
path_diff_candidate_minus_golden: 0
value_diff_golden_minus_candidate: 0
value_diff_candidate_minus_golden: 0
```

The script configures CMake automatically:

- `--compare route` uses `CNOC_QUANT_GOLDEN=OFF` and `ENABLE_CNOC_MFU=OFF`.
- `--quant-cnoc` uses `CNOC_QUANT_GOLDEN=ON` and `ENABLE_CNOC_MFU=ON`.

If a previous run was interrupted, a stale `trace_regression.lock` directory may
remain under the selected build directory.  Remove only that stale lock after
confirming no `NoCDASim` or `run_rtl_trace_regression.sh` process is running.

## 5. RTL Unit Tests

Router-only unit tests build without `ENABLE_CNOC_MFU`:

```bash
bash script/run_rtl_unit_tests.sh --suite router
```

MFU/full unit tests build with `ENABLE_CNOC_MFU`:

```bash
bash script/run_rtl_unit_tests.sh --suite all
bash script/run_rtl_unit_tests.sh --suite mfu
bash script/run_rtl_unit_tests.sh --suite stress --seed 0
```

## 6. Lint Checks

Check both macro configurations before handing changes to synthesis/PPA work:

```bash
verilator --lint-only -DSYNTHESIS -Wno-WIDTHEXPAND \
  -f rtl/router/filelist.f --top-module Router

verilator --lint-only -DSYNTHESIS -DENABLE_CNOC_MFU -Wno-WIDTHEXPAND \
  -f rtl/router/filelist.f --top-module Router
```

The first command is the regular router build.  The second command confirms the
full cNoC/MFU build still elaborates.

## 7. Wrapper Ownership Boundary

The C++ `VerilatedRouter` wrapper bridges NoCDAS and the Verilated RTL model.
It packs flit data/meta/route sideband, drives downstream credit state, steps
the RTL model, and consumes RTL outputs.

The important ownership rule is:

- With `ENABLE_CNOC_MFU=OFF`, the RTL owns only router datapath behavior.  The
  wrapper calls the existing C++ cNoC side-effect model for type4/type5 flits as
  they leave each router.
- With `ENABLE_CNOC_MFU=ON`, the RTL owns cNoC/MFU side effects.  The wrapper
  mirrors RTL output payload/status back into the C++ trace path and MACnet
  waits on RTL cNoC storage progress counters.

Do not use `rtl_router != nullptr` as a proxy for "RTL owns cNoC/MFU".  Use the
wrapper ownership query instead; otherwise route-only RTL runs can deadlock in
cNoC distribution phase gating.
