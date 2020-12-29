module useless(
  input wire clk,
  input wire rst  
);

parameter NS = 2'b00;
parameter UB = 2'b10;
parameter UC = 2'b11;

reg[1:0] state, state_next;

always@(posedge clk or posedge rst) begin
    if(rst) begin
        state = NS;
    end else begin
        state = state_next;
    end
end

always@(*)begin
    
end

endmodule