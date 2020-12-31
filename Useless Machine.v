module useless(
  input clk,
  input rst,
  input mode_sw,
  output DIGIT[3:0],
  output DISPLAY[6:0],
  output LED[15:0],  
);

parameter clk_basic = 16; // for basic modules.
parameter clk_device = 23; // for other devices.
parameter NS = 2'b00;  // Normal Switch.
parameter UB = 2'b10;  // Useless Box.
parameter UC = 2'b11;  // Usefule Car.

wire clk_16, clk_23;
wire mode_db, mode_p; // db for after debounced, p for after one-pulsed.
reg[1:0] state, state_next;
reg[15:0] nums;


// __Clock Divider__ //
clock_divider #(.n(clk_basic)) clk_div_16 (.clk(clk), .clk_div(clk_16));
clock_divider #(.n(clk_device)) clk_div_23(.clk(clk), .clk_div(clk_23));

// __Debounce__ //
debounce mode_sw_db (.clk(clk_16), .pb(mode_sw), .pb_debounced(mode_db));

// __One Pulse__ //
one_pulse mode_1pulse (.clk(clk_16), .pb_debounced(mode_db), .pb_1pulse(mode_p));

// __Seven Segment__ //
seven_segment sev_seg (.clk(clk), .rst(rst), .nums(nums), .display(DISPLAY), .digit(DIGIT));


// __LED__ //
always@(*) begin
   LED[15:0] = 16'b1111_1111_1111_1111;
end

// __State Machine__ //
// __Mode__ //
always@(posedge clk or posedge rst) begin
    if(rst) begin
        state = NS;
    end else if(mode_p) begin
        state = state_next;
    end
end

always@(*)begin
    if(state == NS) begin
        state_next = UB;
    end else if(state == UB) begin
        state_next = UC;
    end else begin
        state_next = NS;
    end
end

endmodule