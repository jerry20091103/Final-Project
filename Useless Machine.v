// customize clock divider.
module clock_divider #(parameter n = 17) (clk, clk_div);
    input clk;
    output clk_div;

    reg [n-1:0] num;
    wire [n-1:0] next_num;

    always @(posedge clk)
    begin
        num = next_num;
    end

    assign next_num = num + 1;
    assign clk_div = num[n-1];

endmodule

module useless(
  /* Basic Signal */
  input clk,
  input rst,
  input mode_sw,           // switch mode between NS, UB, UC
  input[2:0] wanted_ub,      // the mode user wants when in UB mode 
  output DIGIT[3:0],
  output DISPLAY[6:0],
  output LED[15:0],
  /* External Signal */
  input sw0,               // the switch
  input [3:0] ir_sensor,   // 4 IR sensors
  input [1:0] sonic_echo,  // 2 sonic distance sensors
  output [1:0] sonic_trig,
  output relay,            // relay
  output [1:0] motor_cw,   // 2 motors
  output [1:0] motor_ccw,
  output PWM_0,            // 2 servo to flip the switch
  output PWM_1,
  input ble_rx             // bluetooth chip
);

//__Basic Parameter__//
parameter clk_basic = 17; // for basic modules.
parameter NS = 2'b00;  // Normal Switch.
parameter UB = 2'b10;  // Useless Box.
parameter UC = 2'b11;  // Useful Car.

//__Basic Signal__//
wire clk_17;
wire mode_db, mode_p;       // db for after debounced, p for after one-pulsed.
reg[1:0] state, state_next;
wire[1:0] random;

// __Basic Device__ //
clock_divider #(.n(clk_basic)) clk_div_17 (
    .clk(clk), 
    .clk_div(clk_17)
);
debounce mode_sw_db (
    .clk(clk_17), 
    .pb(mode_sw), 
    .pb_debounced(mode_db)
);
onepulse mode_1pulse (.clk(clk_17), 
    .signal(mode_db),
    .op(mode_p)
);

// __Basic Control__ //
always@(*) begin
   LED[15:0] = 16'b1111_1111_1111_1111;
end

assign random = {clk_17, clk}; 

//------------------------------//
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

//--------------------------------//
//__External Signal__//
//__Servo__//
reg servo_enable;
reg servo_sel;
reg [4:0] servo_amount;
reg sw0_final;

//__Motor__//
reg motor_l_enable;
reg motor_l_dir;
reg motor_r_enable;
reg motor_r_dir;

//__Relay__//
wire relay_enable;

//__Sonic Sensor__//
wire [19:0] distance_0;
wire [19:0] distance_1;

//__Bluetooth__//
wire ble_err;
wire [4:0] command;

//__External Device__//
//__Sonic Sensor__//
sonic_top sonic_0(
    .clk(clk),
    .rst(rst),
    .Echo(sonic_echo[0]),
    .Trig(sonic_trig[0]),
    .distance(distance_0)
);
sonic_top sonic_1(
    .clk(clk),
    .rst(rst),
    .Echo(sonic_echo[1]),
    .Trig(sonic_trig[1]),
    .distance(distance_1)
);

//__Servo__//
servo_control servo_ctrl_0(
    .clk(clk),
    .rst(rst),
    .enable(servo_enable),
    .select(servo_sel),
    .amount(servo_amount),
    .PWM_0(PWM_0),
    .PWM_1(PWM_1)
);

//__Motor__//
motor_control motor_ctrl_0(
    .l_enable(motor_l_enable),
    .r_enable(motor_r_enable),
    .l_dir(motor_l_dir),
    .r_dir(motor_r_dir),
    .motor_cw(motor_cw),
    .motor_ccw(motor_ccw)
);

//__Bluetooth__//
bluetooth_control ble_ctrl_m(
    .clk(clk),
    .rst(rst),
    .ble_rx(ble_rx),
    .ble_err(ble_err),
    .switch(command[0]),
    .forward(command[1]),
    .backward(command[2]),
    .left(command[3]),
    .right(command[4])
);

//__External Control__//
//__Bluetooth Control__//
always@(*)begin
    
end

//__Servo Control__//
always@(*)begin
    if((ble_err) || (state == UC)) begin
        servo_enable = 0;
        servo_sel = sw0;
        servo_amount = 0;
    end else begin
        servo_enable = 1;
        if(state == NS) begin
            if(sync_with_sw0[1]) begin
                // do something.
                // do something.
            end else begin
                servo_sel = ~sw0;
                servo_amount = 31;
            end
        end begin
            if(wanted_ub == 3'b000) begin
                if(random == 0) begin
                    // Half cycle
                
                
                end else if(random == 1) begin
                    // One cycle

                end else begin
                    // Dodge
                    servo_sel = sw0;
                    servo_amount = 0;
                end
            end else if(wanted_ub == 3'b001) begin
                
            end else if(wanted_ub == 3'b010) begin
                
            end else if(wanted_ub == 3'b100) begin
                
            end else begin
                
            end
        end
    end
end

//__Motor Control__//
always@(*)begin
    
end


endmodule