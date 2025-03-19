module MCP_tb;

    // Inputs
    reg clk;
    reg reset;

    // Outputs
    wire [2:0] current_state;
    wire [2:0] next_state;
    wire [15:0] PC;
    wire [15:0] IR;
    wire [15:0] ALUresult;
    wire [15:0] memData;
    wire [15:0] writeData;

    // Instantiate the MCP module
    MCP uut (
        .clk(clk),
        .reset(reset),
        .current_state(current_state),
        .next_state(next_state),
        .PC(PC),
        .IR(IR),
        .ALUresult(ALUresult),
        .memData(memData),
        .writeData(writeData)
    );

    // Clock generation
    always #5 clk = ~clk;  // 10 ns clock period

    // Testbench procedure
    initial begin
        // Initialize inputs
        clk = 0;
        reset = 1;

        // Apply reset
        #10 reset = 0;
        #10 reset = 1;

        // Allow some time for the MCP module to execute
        #200;
        
        // Additional stimulus can be applied here for specific instructions
        
        // Finish the simulation
        $stop;
    end

    // Monitor signals for debugging
    initial begin
        $monitor("Time: %0d | State: %b | PC: %h | IR: %h | ALUresult: %h | memData: %h | writeData: %h",
                 $time, current_state, PC, IR, ALUresult, memData, writeData);
    end

endmodule
