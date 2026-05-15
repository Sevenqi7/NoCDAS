# RTL Router Golden Trace Setup

This note only covers how to generate the golden trace and compare it against
the RTL-router trace.

## 1. Build Verilator

If `verilator --version` is missing or points to an old release, build it from
source first:

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

After installation, make sure the new binary is on `PATH`:

```bash
export PATH="$HOME/local/verilator/bin:$PATH"
export VERILATOR_ROOT="$HOME/local/verilator/share/verilator"
verilator --version
```

If this works, put the two `export` lines into `~/.bashrc` or `~/.profile` so
the setting persists.

## 2. Build the project

From the repository root:

```bash
cmake -S . -B build
cmake --build build -j"$(nproc)"
```

## 3. Generate Traces

Run the reference C++ simulator:

```bash
./build/NoCDASim -trace
```

This writes the golden trace, typically `trace.log`.

Then run the RTL-router backed simulator:

```bash
./build/NoCDASim -rtl_router -trace
```

This writes the RTL-router trace, also typically `trace.log`. Copy or rename
the files so both traces are kept.

## 4. Compare Traces

Use the trace comparison script:

```bash
python3 script/compare_traces.py trace_cpp.log trace_rtl.log
```

For the quantized cNoC flow:

```bash
python3 script/compare_traces.py --quant-cnoc trace_cpp_quant.log trace_rtl_quant.log
```

If you prefer the wrapper to generate both traces and compare them in one go,
run:

```bash
bash script/run_rtl_trace_regression.sh
bash script/run_rtl_trace_regression.sh --quant-cnoc
```

## 5. Wrapper Scope

The C++ `VerilatedRouter` wrapper only bridges NoCDAS and the Verilated RTL
router. It packs flit data/meta/route sideband, drives credits/link-ready
state, steps the RTL model, and mirrors RTL output payload back into the C++
trace path. It does not make route, VC, switch, storage, or compute decisions.
