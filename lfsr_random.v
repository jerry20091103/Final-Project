// a pseudo random number generator based on lfsr
// output 2 bit random number
// clk = the generating frequency
// a different number every clock cycle
module lfsr_random(
    input clk,
    input rst,
    output [1:0] random
);

    reg [3:0] state;
    always @(posedge clk, posedge rst) 
    begin
        if(rst)
            state <= 4'b1000;
        else
        begin
            state[2:0] <= state[3:1];
            state[3] <= state[1] ^ state[0];
        end
    end

    assign random[0] = state[0] ^ state[1];
    assign random[1] = state[2] ^ state[3];

endmodule