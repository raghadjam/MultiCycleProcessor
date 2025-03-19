# Multi-Cycle Processor Design

This project involves the development of a Multi-Cycle Processor (MCP) using Verilog. The processor operates with a 16-bit ALU, a Register File with 16 general-purpose registers, Data and Instruction Memory, and a Control Unit.

## Files Included
- MCP.v: The Verilog module for the MCP processor.
- test_MCP.v: The testbench for verifying the MCP processor's functionality.
- AdvancendReport.pdf`: The project report detailing the design, implementation, and testing process.

## Description
The MCP processor is designed as a multi-cycle processor, meaning each instruction is executed over several clock cycles. The processor supports R-type and M-type instructions. It uses a multi-cycle approach, where the execution of each instruction is broken into stages: Fetch, Decode, Execute, Memory, and Write-back. The ALU performs arithmetic and logical operations using a 4-bit opcode, and the Register File provides fast data access. The processor's components work together through proper control signal generation to ensure correct transitions between stages.

Testing has been performed to verify that the processor correctly handles instructions across different stages. The processor is designed to handle instructions efficiently by using multiple cycles for execution.

## How to Run
1. Use any Verilog simulator to compile and simulate the design.
2. Run the testbench (test_MCP.v) to verify the functionality of the processor.


