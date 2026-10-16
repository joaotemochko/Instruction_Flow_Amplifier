# Instruction Flow Amplifier (IFA)

This repository contains the SystemVerilog (RTL) implementation of the **Instruction Flow Amplifier (IFA)**, a microarchitecture mechanism designed to optimize the front-end and instruction dispatch in processors (such as RISC-V and ARM). 

The architecture and concepts implemented here are detailed in the research paper *"Instruction Flow Amplifier: A Novel Architecture for In-Order Processor Front-End Optimization"* (DOI: 10.64552/wipiec.v12i2.139).

## Directory Structure

The project is modularized, and the main source code is located in the `rtl/` directory. The structure consists of the following modules:

* **`ifa_top.sv`**: Top-level module that integrates all subcomponents of the IFA mechanism.
* **`ifa_pkg.sv`**: Package containing structures, constants, and type definitions shared across the entire design.
* **`ifa_pre_decode.sv`**: Pre-decode unit for the initial analysis of the instruction stream.
* **`front_end_buffer.sv`**: Front-end buffer designed to manage temporary storage and smooth out the instruction fetch throughput.
* **`dependency_analysis_unit.sv`**: Unit responsible for dependency analysis, enabling optimized (and potentially out-of-order) dispatch.
* **`dispatcher_unit.sv`**: Dispatch unit that manages sending ready instructions to the execution pipeline.

## Key Features

* Optimization of the instruction fetch stage for in-order processors.
* Mitigation of front-end bottlenecks through run-time analysis.
* Complete RTL (SystemVerilog) implementation ready for simulation and synthesis.

## License

For information regarding use, modification, and distribution, please refer to the `LICENSE` file in the root of this repository.
