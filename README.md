# 🧮 Advanced RISC Microcontroller Design using Verilog

![Microcontroller](https://your-image-link.com)

[![Verilog](https://img.shields.io/badge/Verilog-HDL-blue)](https://en.wikipedia.org/wiki/Verilog)[![VHDL](https://img.shields.io/badge/VHDL-HDL-purple)](https://en.wikipedia.org/wiki/VHDL)[![Quartus Prime](https://img.shields.io/badge/Quartus%20Prime-Intel%20FPGA-green)](https://www.intel.com/content/www/us/en/software/programmable/quartus-prime/overview.html)[![ModelSim](https://img.shields.io/badge/ModelSim-Mentor%20Graphics-red)](https://www.mentor.com/products/fv/modelsim/) [![Xilinx Vivado](https://img.shields.io/badge/Xilinx%20Vivado-FPGA-orange)](https://www.xilinx.com/products/design-tools/vivado.html)  [![Digital Logic](https://img.shields.io/badge/Digital%20Logic-Design-brightgreen)](https://en.wikipedia.org/wiki/Digital_electronics)  [![Embedded Systems](https://img.shields.io/badge/Embedded%20Systems-Development-blue)](https://en.wikipedia.org/wiki/Embedded_system)  [![Computer Architecture](https://img.shields.io/badge/Computer%20Architecture-Hardware-orange)](https://en.wikipedia.org/wiki/Computer_architecture)  [![License](https://img.shields.io/badge/license-MIT-green)](LICENSE)
 
## 🚀 Overview
This project presents a **custom-designed RISC-based microcontroller** implemented using Verilog, featuring a simplified instruction set, Harvard architecture, and non-pipelined execution. The design ensures efficient processing, reduced cycle time, and optimized resource utilization.

## 🎯 Objectives
- Design a **RISC-based** microcontroller using Verilog.
- Implement **Arithmetic Logic Unit (ALU)**, **Control Unit (CU)**, **Registers**, **Multiplexers (MUX)**, **Memory (Program & Data)**, and **Program Counter (PC) Adder**.
- Execute various **instruction sets** and validate the design through simulation.


**🔹 Concepts & Core Skills:**  
- Digital Logic Design  
- FPGA Programming  
- Embedded Systems  
- Microcontroller Design
 
## 🏗️ Architecture & Components
### 1️⃣ **Control Unit (CU)**
Manages the execution flow by decoding instructions and generating control signals.

### 2️⃣ **Arithmetic Logic Unit (ALU)**
Performs arithmetic operations (addition, subtraction) and logical operations (AND, OR, XOR).

### 3️⃣ **Registers**
Stores intermediate and final computation results for efficient data handling.

### 4️⃣ **Program Counter (PC) Adder**
Increments the program counter to fetch the next instruction.

### 5️⃣ **Multiplexers (MUX1 & MUX2)**
Selects appropriate input data for ALU and memory operations.

### 6️⃣ **Memory (Program & Data)**
- **Program Memory (PMem):** Stores instruction codes.
- **Data Memory (DMem):** Stores data required for operations.

## 📜 Instruction Set
### 🔄 **Data Transfer Group**
- `MOVAM`: Move accumulator value to memory.
- `MOVMA`: Move memory value to accumulator.
- `MOVIA`: Move immediate value to accumulator.

### ➕ **Arithmetic Group**
- `ADD`: Perform addition.
- `SUB`: Perform subtraction.
- `INC`: Increment accumulator value.
- `DEC`: Decrement accumulator value.

### 🔣 **Logical Group**
- `AND`: Logical AND operation.
- `OR`: Logical OR operation.
- `XOR`: Logical XOR operation.

### 🔁 **Branching Group**
- `JMP`: Unconditional jump.
- `JNZ`: Jump if zero flag is not set.

### ⚙ **Machine Control Group**
- `NOP`: No operation.

## 🛠️ Implementation using Verilog
This microcontroller is implemented using Verilog HDL, structured into modular components:

- `control_unit.v`: Implements control logic.
- `alu.v`: Defines arithmetic and logical operations.
- `registers.v`: Manages data storage.
- `mux.v`: Implements multiplexer logic.
- `memory.v`: Defines program and data memory.
- `pc_adder.v`: Handles program counter incrementation.
- `testbench.v`: Simulates and verifies the design.
## 📂 Featured Code
### 🔹 Control Unit (CU)
```verilog
module ControlUnit (
    input [1:0] stage,
    input [11:0] IR,
    output reg PC_Enable, Acc_Enable, ALU_Enable,
    output reg [3:0] ALU_Mode
);

always @(*) begin
    {PC_Enable, Acc_Enable, ALU_Enable} = 3'b000;
    ALU_Mode = 4'd0;

    case (stage)
        2'b01: PC_Enable = 1;
        2'b10: Acc_Enable = 1;
        2'b11: begin
            ALU_Enable = 1;
            ALU_Mode = IR[10:8];
        end
    endcase
end
endmodule
```

### 🔹 Arithmetic Logic Unit (ALU)
```verilog
module ALU (
    input [7:0] Operand1, Operand2,
    input [3:0] Mode,
    output reg [7:0] Result
);

always @(*) begin
    case (Mode)
        4'b0000: Result = Operand1 + Operand2;
        4'b0001: Result = Operand1 - Operand2;
        4'b0010: Result = Operand1 & Operand2;
        4'b0011: Result = Operand1 | Operand2;
        default: Result = 8'd0;
    endcase
end
endmodule
```

### 🔹 Multiplexer (MUX)
```verilog
module MUX (
    input [7:0] In0, In1,
    input Sel,
    output [7:0] Out
);
assign Out = Sel ? In1 : In0;
endmodule
```

### 🔹 Program Counter (PC) Adder
```verilog
module Adder (
    input [7:0] In,
    output [7:0] Out
);
assign Out = In + 1;
endmodule
```

## ✅ Verification & Simulation
### 🔬 Sample Test 1: Finding Maximum of Three Numbers
- Compares three numbers (e.g., `5, 12, 2`) and stores the largest in the accumulator.
- **Simulation Results:** Verified using Verilog testbench.

### 🔬 Sample Test 2: Logical and Arithmetic Operations
- Performs a sequence of logical and arithmetic operations.
- **Simulation Results:** Correct values stored in Data Memory (DMem).

## 📌 Methodology
1. **Design & Planning:** Define architecture and instruction set.
2. **Implementation:** Code microcontroller modules in Verilog.
3. **Simulation & Testing:** Verify correctness using sample test cases.
4. **Optimization:** Improve efficiency and reduce hardware cost.
5. **Final Verification:** Validate through comprehensive simulations.

## 📄 Future Enhancements
- **Pipeline Execution:** Improve performance through instruction pipelining.
- **Enhanced ALU Operations:** Support additional arithmetic/logical functions.
- **Interrupt Handling Mechanism:** Implement for real-time applications.

## 📁 Project Files
```
📂 risc_microcontroller
 ├── 📜 README.md  # This file
 ├── 📂 src  # Source Code
 │   ├── control_unit.v
 │   ├── alu.v
 │   ├── registers.v
 │   ├── mux.v
 │   ├── memory.v
 │   ├── pc_adder.v
 │   └── testbench.v
 ├── 📂 docs  # Documentation
 │   ├── architecture_diagram.png
 │   └── instruction_set.pdf
 ├── 📂 simulations  # Test results
 │   ├── sample_test1.vcd
 │   ├── sample_test2.vcd
 │   └── waveform.png
 ├── LICENSE  # License file
 ├── .gitignore  # Git ignore file
```

## 📜 References
- Harvard Architecture: [Wikipedia](https://en.wikipedia.org/wiki/Harvard_architecture)
- Verilog HDL: [IEEE Standard 1364-2005](https://standards.ieee.org/standard/1364-2005.html)
- RISC Concept: [Computer Science Guide](https://en.wikipedia.org/wiki/Reduced_instruction_set_computer)

## 🏆 Conclusion
This project successfully implements a **custom RISC-based microcontroller** in Verilog. The architecture, instruction set, and simulation results validate the functionality and efficiency of the design. Future enhancements can further optimize the microcontroller’s performance.

---

🚀 **Designed & Developed by:** ahtishamsudheer@gmail.com

🌟 If you found this project useful, don’t forget to give it a ⭐ on GitHub!
