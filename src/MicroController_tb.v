`timescale 1ns/10ps

module MicroController_tb;

    reg clk;              // Positive edge triggered clock
    reg rst;              // Active high reset
    
    // Internal signals from the DUT (Device Under Test)
    wire [7:0] DI;       // Data Input to DMem
    reg E;                // ALU Enable
    wire [7:0] ALU_Out;  // ALU Output
    wire [7:0] Acc;      // Accumulator
    wire [3:0] SR;       // Status Register
    wire [11:0] instr_set; // Instruction set
    
    // Instantiate the MicroController (DUT)
    MicroController UUT(
        .clk(clk),
        .rst(rst)
    );

    // Clock generation (period of 10ns)
    always #5 clk = ~clk;

    // Testbench stimulus
    initial begin
        // Initialize clock and reset
        $dumpfile("MicroController_tb.vcd");
        $dumpvars(0, MicroController_tb);
        clk = 0;      
        rst = 1;       // Apply reset
        #20 rst = 0;   // Release reset
        #980 $finish;  // Run for some time and finish
    end

endmodule
