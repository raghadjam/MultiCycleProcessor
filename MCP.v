module MCP(
    input clk,
    input reset,
    output reg [2:0] current_state, // Output current state for waveform
    output reg [2:0] next_state,    // Output next state for waveform
    output reg [15:0] PC,           // Output PC for waveform
    output reg [15:0] IR,           // Output Instruction Register for waveform
    output reg [15:0] ALUresult,    // Output ALU result for waveform
    output reg [15:0] memData,      // Output memory data for waveform
    output reg [15:0] writeData     // Output data to be written back for waveform	

);

    // Define states for the FSM using parameters
    parameter FETCH      = 3'b000;
    parameter DECODE     = 3'b001;
    parameter EXECUTE    = 3'b010;
    parameter MEM_ACCESS = 3'b011;
    parameter WRITE_BACK = 3'b100;

    // Control signals
    reg RegWr, MemRd, MemWr;

    // Decode signals
    reg [3:0] opCode;
    reg [3:0] Rd, Rs1, Rs2;

    // Register File connections
    wire [15:0] readData1, readData2;

    // ALU connections
    wire [2:0] ALUop;

    // Instruction Memory Instance
    instructionMem IM(
        .PC(PC),
        .instruction(instruction)
    );

    // Data Memory Instance
    dataMem dm(
        .in_data(readData2),
        .address(Rd),
        .clk(clk),
        .MemRd(MemRd),
        .MemWr(MemWr)
    );

    // Register File Instance
    RegisterFile RF(
        .clk(clk),
        .read_port1(Rs1),
        .read_port2(Rs2),
        .write_port(Rd),
        .writeEn(RegWr),
        .writeData(writeData),
        .readData1(readData1),
        .readData2(readData2)
    );

    // Control Unit Instance
    controlUnit CU(
        .opCode(opCode),
        .RegWr(RegWr),
        .MemRd(MemRd),
        .MemWr(MemWr),
        .ALUop(ALUop)
    );
	 
	
    // FSM State transitions
    always @(posedge clk or negedge reset) begin
        if (!reset)
            current_state <= FETCH;
        else
            current_state <= next_state;
    end

    // FSM Control Logic: Determine next state
    always @(*) begin
        case (current_state)
            FETCH:    begin  
			next_state = DECODE;  
			end
            DECODE: begin
                if (opCode <= 4'b0100) //  R-Type opcode
                    next_state = EXECUTE;
                else if (MemRd || MemWr)
                    next_state = MEM_ACCESS;
                else
                    next_state = FETCH;
            end
            EXECUTE: begin
                if (opCode <= 4'b0100) // R-Type instruction
                    next_state = WRITE_BACK;
                else
                    next_state = FETCH;
            end
            MEM_ACCESS: begin
                if (MemRd)	//load
                    next_state = WRITE_BACK;
                else if (MemWr)	//store
                    next_state = FETCH;
                else
                    next_state = FETCH;
            end
            WRITE_BACK: next_state = FETCH;
            default:    next_state = FETCH;
        endcase
    end

    // FETCH Stage
    always @(posedge clk or negedge reset) begin
        if (!reset)
			begin
            PC <= 16'b0;
		   IR <= 16'b0;
		  	end
        else if (current_state == FETCH) 
			begin
            IR <= IM.instruction; // Fetch instruction
		   PC <= PC + 16'd2; 
		   end
    end

    // DECODE Stage
    always @(*) begin
        if (current_state == DECODE) begin
            opCode = IR[15:12];
            Rd = IR[11:8];
            Rs1 = IR[7:4];
            Rs2 = IR[3:0];
        end
    end

    // EXECUTE Stage
    always @(*) begin
        if (current_state == EXECUTE) begin
            case (ALUop)
                3'b000: ALUresult = readData1 + readData2;
                3'b001: ALUresult = readData1 - readData2;
                3'b010: ALUresult = readData1 & readData2;
                3'b011: ALUresult = readData1 | readData2;
                3'b100: ALUresult = readData1 ^ readData2;
                default: ALUresult = 16'b0;
            endcase
        end
    end
always @(posedge clk) begin
    if (current_state == MEM_ACCESS) begin
        if (MemRd) begin
            memData <= dm.dataMemory[RF.register[Rs1]];  // Load data
            $display("LOAD: Address = %h, Data = %h", RF.register[Rs1], memData);
        end
        if (MemWr) begin
            dm.dataMemory[RF.register[Rs1]] <= RF.register[Rd];  // Store data
            $display("STORE: Address = %h, Data = %h", RF.register[Rs1], RF.register[Rd]);
        end
    end
end
// WRITE-BACK Stage
always @(posedge clk) begin
    if (current_state == WRITE_BACK) begin
        if (RegWr) begin
            if (MemRd) begin
                writeData <= memData;  // Write loaded data to Reg[Rd]
            end else begin
                writeData <= ALUresult;  // Write ALU result to Reg[Rd]
            end
            RF.register[Rd] <= writeData;  // Write-back to register
        end
    end
end


endmodule 


//************************************************RegisterFile***********************************************************************************//


module RegisterFile(
	input clk,
	input[3:0] read_port1, read_port2, write_port,
	input writeEn,
	input [15:0] writeData,
	output [15:0] readData1, readData2
	);	   
	reg [15:0] register [15:0] = '{0, 1, 2, 3, 7,				  
								  6, 4, 9, 8 , 1 , 5 , 6,3 ,4 ,6,9};
 
	assign readData1 = register[read_port1];
	assign readData2 = register[read_port2];
	
	always @(posedge clk) 
	begin
		if(writeEn)
			register[write_port] <= writeData;
	end			 
	endmodule
	
	
//************************************************data Memory***********************************************************************************//
	
module dataMem(
	input [15:0] in_data,
	input [7:0] address,
	input clk,
	input MemRd,
	input MemWr
	);	
	integer i;
	
	reg [15:0] dataMemory [0:255];
	initial begin
	for(i=0; i<=255; i++)
				  dataMemory[i] = 16'b0;
end
	
	always @ (posedge clk)
		begin
            if(MemWr)
				dataMemory[address] <= in_data;	
		 end
		 endmodule 
		 
	
//************************************************control Unit***********************************************************************************//
module controlUnit (
    input [3:0] opCode,
    output reg RegWr, MemRd, MemWr,
    output reg [2:0] ALUop
);

    always @(*) 
	begin
        case(opCode)
            4'b0000:
			begin 
                RegWr = 1;
                MemRd = 0;
                MemWr = 0;
                ALUop = 3'b000;
            end
            4'b0001: 
			begin 
                RegWr = 1;
                MemRd = 0;
                MemWr = 0;
                ALUop = 3'b001;
            end
            4'b0010: 
			begin 
                RegWr = 1;
                MemRd = 0;
                MemWr = 0;
                ALUop = 3'b010;
            end
            4'b0011: 
			begin 
                RegWr = 1;
                MemRd = 0;
                MemWr = 0;
                ALUop = 3'b011;
            end
            4'b0100: begin 
                RegWr = 1;
                MemRd = 0;
                MemWr = 0;
                ALUop = 3'b100;
            end
            4'b0101: begin 
                RegWr = 1;
                MemRd = 1;
                MemWr = 0;
                ALUop = 3'bxxx; 
            end
            4'b0110: 
			begin 
                RegWr = 0;
                MemRd = 0;
                MemWr = 1;
                ALUop = 3'bxxx; 
            end
            default:
			begin 
                RegWr = 0;
                MemRd = 0;
                MemWr = 0;
                ALUop = 3'b000;
            end
            endcase
            end

endmodule

	
//************************************************instruction Memory***********************************************************************************//
module instructionMem(
    input [15:0] PC,        
    output reg [15:0] instruction   
);

   
    reg [15:0] memory [0:255];
				 
   initial begin
    memory[0]  = 16'b0110001100100011; 
    memory[1]  = 16'b0001010001100101; 
    memory[2]  = 16'b0010000100110010; 
    memory[3]  = 16'b0110000100100110; 
    memory[4]  = 16'b0100000101100101; 
    memory[5]  = 16'b0110001000110000; 
    memory[6]  = 16'b0110001010000011; 
    memory[7]  = 16'b0111000000010001; 
    memory[8]  = 16'b0000000000101000;
    memory[9]  = 16'b0110000000110111;
    memory[10] = 16'b0010000001000100; 
    memory[11] = 16'b0011000001010001; 
    memory[12] = 16'b0100000010001100; 
    memory[13] = 16'b0101000010010110; 
    memory[14] = 16'b0110000010100001;
    memory[15] = 16'b0110000010110110;
end

   
    always @ (PC) begin
        instruction = memory[PC[7:0]]; 
    end

endmodule	


	
//************************************************Test Bench***********************************************************************************//
module test_MCP;

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
        reset = 0;

        // Apply reset
        #10 reset = 1;
		
    // Initialize data memory with test values
    uut.dm.dataMemory[0] = 16'h0001;
    uut.dm.dataMemory[1] = 16'h0002;
    uut.dm.dataMemory[2] = 16'h0003;
    uut.dm.dataMemory[3] = 16'h0004;
    uut.dm.dataMemory[4] = 16'h0005;
    uut.dm.dataMemory[5] = 16'h0006;
    uut.dm.dataMemory[6] = 16'h0007;
    uut.dm.dataMemory[7] = 16'h0008;
    // Add more initial values if needed


        // Populate the instruction memory with more memory operations
        uut.IM.memory[0] = 16'b0101000100100001;  // LOAD R2, [R1] (Load from address in R1)
        uut.IM.memory[1] = 16'b0110001000110010;  // STORE R3, [R2] (Store to address in R2)
        uut.IM.memory[2] = 16'b0101001100100011;  // LOAD R4, [R3] (Load from address in R3)
        uut.IM.memory[3] = 16'b0110001100110100;  // STORE R5, [R4] (Store to address in R4)
        uut.IM.memory[4] = 16'b0101010000100101;  // LOAD R6, [R5] (Load from address in R5)
        uut.IM.memory[5] = 16'b0110010000110110;  // STORE R7, [R6] (Store to address in R6)
        uut.IM.memory[6] = 16'b0101010100100111;  // LOAD R8, [R7] (Load from address in R7)
        uut.IM.memory[7] = 16'b0110010100111000;  // STORE R9, [R8] (Store to address in R8)

        // Allow some time for the MCP module to execute
        #400;

        // Finish the simulation
        $stop;
    end

    // Monitor signals for debugging
    initial begin
        $monitor("Time: %0d | State: %b | PC: %h | IR: %h | ALUresult: %h | memData: %h | writeData: %h",
                 $time, current_state, PC, IR, ALUresult, memData, writeData);
    end

endmodule 