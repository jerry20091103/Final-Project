// NOTICE:
// You may search "NOTICE" for detailed/important information in this code.
// WARNING:
// You may search "WARNING" for the problems (now solved) in old code.

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
  input clk_100,
  input rst,                // btnU
  input mode_sw,           // switch mode between NS, UB, UC btnC
  input [2:0] wanted_ub,      // the mode user wants when in UB mode sw 0 1 2 
  output [3:0] DIGIT,
  output [6:0] DISPLAY,
  output reg [15:0] LED,
  /* External Signal */
  input sw0,               // the switch
  input [3:0] ir_sensor,   // 4 IR sensors
  input [1:0] sonic_echo,  // 2 sonic distance sensors
  output [1:0] sonic_trig,
  output reg relay,            // relay
  output [1:0] motor_cw,   // 2 motors
  output [1:0] motor_ccw,
  output PWM_0,            // 2 servo to flip the switch
  output PWM_1,
  input ble_rx             // bluetooth chip
);

//__Basic Parameter__//
parameter clk_basic = 17;
parameter clk_led = 22;
parameter NS = 2'b00;  // Normal Switch.
parameter UB = 2'b10;  // Useless Box.
parameter UC = 2'b11;  // Useful Car.

// NOTICE:
// wanted_ub = 3'b000 -> random mode 
// wanted_ub = 3'b001 -> classic mode (when user toggle the switch, the box turn it back)
// wanted_ub = 3'b010 -> dodge mode (the box run away and dodge)
// wanted_ub = 3'b100 -> advance mode (when user tries to toggle the switch, the box also tries to 
//                                     toggle the switch by itself and will immediately turn it back)
// wanted_ub = none of above -> cause error

//__Basic Signal__//
wire clk_17, clk_22, clk;
wire mode_db, mode_p;       // db for after debounced, p for after one-pulsed.
reg[1:0] state, state_next;
reg[15:0] LED_next;

wire [6:0] d0;
wire [6:0] d1;
wire [6:0] d2;
wire [6:0] d3;

reg [3:0] digit_0; 
reg [3:0] digit_1; 
reg [3:0] digit_2; 
reg [3:0] digit_3; 

wire [4:0] command;
wire ble_err;

// __Basic Device__ //
clock_divider #(.n(clk_basic)) clk_div_17 (
    .clk(clk_100), 
    .clk_div(clk_17)
);

clock_divider #(.n(clk_led)) clk_div_23 (
    .clk(clk_100), 
    .clk_div(clk_22)
);

clock_divider #(.n(1)) clk_div_50 (
    .clk(clk_100), 
    .clk_div(clk)
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

led_display dis(
    .digit_0(d0),
    .digit_1(d1),
    .digit_2(d2),
    .digit_3(d3),
    .clk(clk_17),
    .digit_sel(DIGIT),
    .display(DISPLAY)
);

int_to_7_bit d0_c(
    .digit_in(digit_0),
    .digit_out(d0)
);
int_to_7_bit d1_c(
    .digit_in(digit_1),
    .digit_out(d1)
);
int_to_7_bit d2_c(
    .digit_in(digit_2),
    .digit_out(d2)
);
int_to_7_bit d3_c(
    .digit_in(digit_3),
    .digit_out(d3)
);

// __Basic Control__ //
always@(posedge clk_22 or posedge rst) begin
    if(rst) begin
        LED[0] = 1;
        LED[15:1] = 15'd0;
    end else begin
        LED = LED_next;
    end
end

always@(*) begin
   if(state == NS) begin
       // led will flash if sw0 is on
       if(sw0 && clk_22) begin
           LED_next = 16'b1111_1111_1111_1111;
       end else begin
           LED_next = 16'b0000_0000_0000_0000;
       end
   end else if(state == UB) begin
       // show wanted ub
       LED_next[0] = 0;
       LED_next[3:1] = wanted_ub;
       LED_next[15:4] = 12'd0;
   end else begin
       if(command[3]) begin
           // left
           if(LED == 16'b0000_0000_0000_0001) begin
               LED_next = 16'b1000_0000_0000_0000;
           end else begin
               LED_next = LED >> 1'b1;
           end
       end else if(command[4]) begin
           // right
           if(LED == 16'b1000_0000_0000_0000) begin
               LED_next = 16'b0000_0000_0000_0001;
           end else begin
               LED_next = LED << 1'b1;
           end
       end else begin
           // forward and backward
           LED_next[6:0] = 7'd0;
           LED_next[7] = 1;
           LED_next[15:8] = 8'd0;
       end
   end
end

always@(*) begin
    digit_0 = state;
    if((state == NS) || (state == UC)) begin
        if(ble_err) begin
            // 11: E
            // 12: r
            digit_1 = 12;
            digit_2 = 12;
            digit_3 = 11;
        end else begin
            // 15: display nothing
            digit_1 = 15;
            digit_2 = 15;
            digit_3 = 15;
        end
    end else begin
        if((wanted_ub == 3'b111) || (wanted_ub == 3'b110) 
            || (wanted_ub == 3'b101) || (wanted_ub == 3'b011)) begin
                digit_1 = 12;
                digit_2 = 12;
                digit_3 = 11;
        end else begin
            digit_1 = 15;
            digit_2 = 15;
            digit_3 = 15;        
        end
    end
end

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
//__Random__//
reg[1:0] random, random_next;

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

//__IR Sensor__//
wire [3:0] ir_sensor_deb;

//__Bluetooth__//
// declaration of wire ble_err; is moved up
// declaration of wire [4:0] command; is moved up

//__External Device__//
//__Sonic Sensor__//
sonic_top sonic_0(
    .clk(clk_100),
    .rst(rst),
    .Echo(sonic_echo[0]),
    .Trig(sonic_trig[0]),
    .distance(distance_0)
);
sonic_top sonic_1(
    .clk(clk_100),
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

//__IR Sensor__//
// ir_sensor = 1 when no object infront, = 0 when detected object
debounce ir_0_deb(
    .pb(~ir_sensor[0]),
    .pb_debounced(ir_sensor_deb[0]),
    .clk(clk_17)
);
debounce ir_1_deb(
    .pb(~ir_sensor[1]),
    .pb_debounced(ir_sensor_deb[1]),
    .clk(clk_17)
);
debounce ir_2_deb(
    .pb(~ir_sensor[2]),
    .pb_debounced(ir_sensor_deb[2]),
    .clk(clk_17)
);
debounce ir_3_deb(
    .pb(~ir_sensor[3]),
    .pb_debounced(ir_sensor_deb[3]),
    .clk(clk_17)
);

//__External Control__//
//__Toggle Control__//
always@(posedge clk or posedge rst)begin
    if(rst) begin
        sw0_final = sw0;
    end else begin
        if(state == NS) begin
            if(!ble_err && command[0]) begin
                sw0_final = ~sw0_final;
            end
        end
    end
end

//__Random Control__//
always@(posedge clk) begin
    random = random_next;
end
// NOTICE:
// The problems of random are solved, by only change random when user isn't nearby.

/* Old random generator
// WARNING:
// 1. This random does not use LFSR, but take clk as result.
// 2. Random stays the same when state = UB. This is because UB state needs to check this
//    signal to make servo work correctly. If random has changed during the process, it
//    might cause some error. You may modify these if you have a better solution.
// 3. random = 0 -> advance mode
//    random = 1 -> classic mode
//    random = 2, 3 -> dodge mode
always@(*) begin
    if(state == UB) begin
        random_next = random;
    end else begin
        random_next = clk_17 * 2'd2 + clk;
    end
end
*/
//__Servo Control__//
// NOTICE:
// 1. good_dis is the distance when user isn't near the box.
//    Usually it should be the length of the box, but you can deine your own good_dis as well.
//    In that case, this "closer effect" may not be suitable.
// 2. delta = left - right. (left is longer)
//    This is because our box is not symmetric. If it is symmetric, it should be 0.
// 3. For more detailed information, check "closer effect" section below.
`define good_dis 20 
`define delta 4      
always@(*)begin
    if(sw0 == sw0_final) begin
        if(state == UB) begin
            servo_enable = 1;
            if(wanted_ub == 3'b000) begin
                if(random) begin
                    // no effect.
                    random_next = clk_17 * 2'd2 + clk;
                    servo_sel = sw0;
                    servo_amount = 0;
                end else begin
                    // "closer effect"
                    // -> The hand lift higher when user get closer.
                    // Take min(distance_0, distance_1 + `delta) as accurate distance.
                    // Decide servo_amount according to this "accurate distance".
                    servo_sel = ~sw0;
                    if(ir_sensor_deb) begin
                        random_next = random;
                        servo_amount = 31;
                    end else begin
                        if((distance_0 > `good_dis) && (distance_1 + `delta > `good_dis)) begin
                            random_next = clk_17 * 2'd2 + clk;
                            servo_amount = 0;
                        end else if(distance_0 > distance_1 + `delta) begin
                            random_next = random;
                            servo_amount = ((distance_1 + `delta) * 3 < 25) ?
                                                (distance_1 + `delta) * 3 : 31; 
                        end else begin
                            random_next = random;
                            servo_amount = (distance_0 * 3 < 25) ? distance_0 * 3 : 31;
                        end
                    end
                    // Old Algorithm
                    // WARNING: This is not perfect due to the following problem.
                    // Problem 1: Only use the distance of the sonic sensor on the normal side,
                    //            but user may come from the other side, so the distance isn't precise.
                    // Problem 2: If user comes from the other side of sw0, servo_amount will be
                    //            more than the normal side.
                    /*
                    if(sw0) begin 
                        if(ir_sensor_deb) begin
                            servo_amount = 31;
                        end else if(distance_0 > `good_dis) begin
                            servo_amount = 0; 
                        end else begin
                            servo_amount = distance_0 + 25 - `good_dis;
                        end
                    end else begin
                        if(ir_sensor_deb) begin
                            servo_amount = 31;
                        end else if(distance_1 + `delta > `good_dis) begin
                            servo_amount = 0; 
                        end else begin
                            servo_amount = distance_1 + `delta + 25 - `good_dis;
                        end
                    end
                    */
                end
            end else if(wanted_ub == 3'b001) begin
                random_next = clk_17 * 2'd2 + clk;
                servo_sel = sw0;
                servo_amount = 0;
            end else if(wanted_ub == 3'b010) begin
                random_next = clk_17 * 2'd2 + clk;
                servo_sel = sw0;
                servo_amount = 0;
            end else if(wanted_ub == 3'b100) begin
                // "closer effect"
                servo_sel = ~sw0;
                if(ir_sensor_deb) begin
                    random_next = random;
                    servo_amount = 31;
                end else begin
                    if((distance_0 > `good_dis) && (distance_1 + `delta > `good_dis)) begin
                        random_next = clk_17 * 2'd2 + clk;
                        servo_amount = 0;
                    end else if(distance_0 > distance_1 + `delta) begin
                        random_next = random;
                        servo_amount = ((distance_1 + `delta) * 3 < 25) ?
                                            (distance_1 + `delta) * 3 : 31; 
                    end else begin
                        random_next = random;
                        servo_amount = (distance_0 * 3 < 25) ? distance_0 * 3 : 31;
                    end
                end
                /*  Old Algorithm
                if(sw0) begin
                    if(ir_sensor_deb) begin
                        servo_amount = 31;
                    end else if(distance_0 > `good_dis) begin
                        servo_amount = 0; 
                    end else begin
                        servo_amount = distance_0 + (25 - `good_dis);
                    end
                end else begin
                    if(ir_sensor_deb) begin
                        servo_amount = 31;
                    end else if(distance_1 > `good_dis) begin
                        servo_amount = 0; 
                    end else begin
                        servo_amount = distance_1 + (25 - `good_dis);
                    end
                end
                */
            end else begin
                // user wants two kinds of ub and cause error.
                random_next = clk_17 * 2'd2 + clk;
                servo_sel = sw0;
                servo_amount = 0;
            end
        end else begin
            // Do nothing in NS and UC mode.
            random_next = clk_17 * 2'd2 + clk;
            servo_enable = 0;
            servo_sel = sw0;
            servo_amount = 0;
        end
    end else begin
        servo_enable = 1;
        random_next = random;
        if(state == NS) begin
            servo_sel = ~sw0;
            servo_amount = 31;
        end else if(state == UB) begin
            if(wanted_ub == 3'b000) begin
                if(random < 2) begin
                    // Use servo.
                    servo_sel = ~sw0;
                    servo_amount = 31;
                end else begin
                    // Don't use servo.
                    servo_sel = sw0;
                    servo_amount = 0;
                end
            end else if(wanted_ub == 3'b001) begin
                servo_sel = ~sw0;
                servo_amount = 31;
            end else if(wanted_ub == 3'b010) begin
                servo_sel = sw0;
                servo_amount = 0;
            end else if(wanted_ub == 3'b100) begin
                servo_sel = ~sw0;
                servo_amount = 31;
            end else begin
                servo_sel = sw0;
                servo_amount = 0;
            end
        end else begin
            servo_sel = sw0;
            servo_amount = 0;
        end
    end
end

//__Motor Control__//
always@(*)begin
    if(state == NS) begin
        motor_l_enable = 0;
        motor_r_enable = 0;
        motor_l_dir = 0;
        motor_r_dir = 0;
    end else if(state == UB) begin
        if(wanted_ub == 3'b000) begin
            if(random < 2) begin
                motor_l_enable = 0;
                motor_r_enable = 0;
                motor_l_dir = 0;
                motor_r_dir = 0;
            end else begin
                // NOTICE:
                // Two possible situation to stop:
                // 1. Very close to sensor -> this means away from sw0 after cars moved.
                // 2. Very far from sensor -> this means user isn't near the box.
                if(((distance_0 < `good_dis / 3 + 1) || (distance_1 + `delta < `good_dis / 3 + 1))
                    || ((distance_0 > `good_dis / 2) && (distance_1 + `delta > `good_dis / 2))) begin
                    motor_l_enable = 0;
                    motor_r_enable = 0;
                    motor_l_dir = 0;
                    motor_r_dir = 0;
                end else if(distance_0 > distance_1 + `delta) begin
                    // approximately closer to right
                    motor_l_enable = 1;
                    motor_r_enable = 1;
                    motor_l_dir = 0;
                    motor_r_dir = 0;
                end else begin
                    // approximately closer to left
                    motor_l_enable = 1;
                    motor_r_enable = 1;
                    motor_l_dir = 1;
                    motor_r_dir = 1;
                end
            end
        end else if(wanted_ub == 3'b010) begin
            if(((distance_0 < `good_dis / 3 + 1) || (distance_1 + `delta < `good_dis / 3 + 1))
                || ((distance_0 > `good_dis / 2) && (distance_1 + `delta > `good_dis / 2))) begin
                motor_l_enable = 0;
                motor_r_enable = 0;
                motor_l_dir = 0;
                motor_r_dir = 0;
            end else if(distance_0 > distance_1 + `delta) begin
                // approximately closer to right (not symmetric)
                motor_l_enable = 1;
                motor_r_enable = 1;
                motor_l_dir = 0;
                motor_r_dir = 0;
            end else begin
                // approximately closer to left (not symmetric)
                motor_l_enable = 1;
                motor_r_enable = 1;
                motor_l_dir = 1;
                motor_r_dir = 1;
            end
        end else begin
            motor_l_enable = 0;
            motor_r_enable = 0;
            motor_l_dir = 0;
            motor_r_dir = 0;
        end
    end else begin
        if(!ble_err) begin
            if(command[1]) begin
                // forward
                motor_l_enable = 1;
                motor_r_enable = 1;
                motor_l_dir = 1;
                motor_r_dir = 1;
            end else if(command[2]) begin
                // backward
                motor_l_enable = 1;
                motor_r_enable = 1;
                motor_l_dir = 0;
                motor_r_dir = 0;
            end else if(command[3]) begin
                // left
                motor_l_enable = 0;
                motor_r_enable = 1;
                motor_l_dir = 0;
                motor_r_dir = 1;
            end else if(command[4]) begin
                // right
                motor_l_enable = 1;
                motor_r_enable = 0;
                motor_l_dir = 1;
                motor_r_dir = 0;
            end else begin
                // no command
                motor_l_enable = 0;
                motor_r_enable = 0;
                motor_l_dir = 0;
                motor_r_dir = 0;
            end
        end else begin
            // error
            motor_l_enable = 0;
            motor_r_enable = 0;
            motor_l_dir = 0;
            motor_r_dir = 0;
        end
    end
end

//__Relay Control__//
always@(*) begin
    if(state == NS) begin
        relay = sw0;
    end else begin
        relay = 0;
    end
end

endmodule