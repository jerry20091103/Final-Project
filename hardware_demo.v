// A demo file to test hardware function
module hardware_demo(
    input clk,
    // pushbutton
    input rst,            // btnC
    // switches
    input servo_enable,   // sw0 enable fliping arms
    input motor_l_enable, // sw1
    input motor_l_dir,    // sw2
    input motor_r_enable, // sw3
    input motor_r_dir,    // sw4
    input relay_enable,    // sw5
    // external devices 
    input sw0,              // the switch
    input [3:0] ir_sensor,  // 4 IR sensors
    input [1:0] sonic_echo, // 2 sonic distance sensors
    output [1:0] sonic_trig,
    output relay,           // relay
    output [1:0] motor_cw,   // 2 motors
    output [1:0] motor_ccw,
    output PWM_0,           // 2 servo to flip the switch
    output PWM_1,
    // displays
    output [15:0] led,     // show ir sensor status (0~3)
    output [6:0] DISPLAY,  // digit 0, 1 -> sonic_0 distance (cm)
    output [3:0] DIGIT     // digit 2, 3 -> sonic_1
);
    // clocks
    wire clk_17;
    clock_divider_17 clk_div_17_01(
        .clk(clk),
        .clk_div(clk_17)
    );

    // regs
    wire [19:0] distance_0;
    wire [19:0] distance_1;
    reg servo_sel;
    reg [4:0] servo_amount;

    // connect sonic sensors
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

    // connect servos
    servo_control servo_ctrl_0(
        .clk(clk),
        .rst(rst),
        .enable(servo_enable),
        .select(servo_sel),
        .amount(servo_amount),
        .PWM_0(PWM_0),
        .PWM_1(PWM_1)
    );

    // connect motors
    motor_control motor_ctrl_0(
        .l_enable(motor_l_enable),
        .r_enable(motor_r_enable),
        .l_dir(motor_l_dir),
        .r_dir(motor_r_dir),
        .motor_cw(motor_cw),
        .motor_ccw(motor_ccw)
    );

    // flip switches
    always @(*) 
    begin
        if(servo_enable)
        begin
            if(sw0)
            begin
                servo_sel = 0;
                servo_amount = 31;
            end
            else 
            begin
                servo_sel = 1;
                servo_amount = 31;
            end
        end 
        // sel and amount should be useless in this case
        else
        begin
            servo_sel = 0;
            servo_amount = 0;
        end   
    end

    // debounce and reverse ir sensors
    wire ir_sensor_deb [3:0];
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

    // assign relay output
    assign relay = relay_enable;

    // show status of "the switch"
    assign led[15] = sw0;

    // show IR status on leds
    assign led[0] = ir_sensor_deb[0];
    assign led[1] = ir_sensor_deb[1];
    assign led[2] = ir_sensor_deb[2];
    assign led[3] = ir_sensor_deb[3];

    // show distance on display
    wire [6:0] d0;
    wire [6:0] d1;
    wire [6:0] d2;
    wire [6:0] d3;
    led_display dis(
        .digit_0(d0),
        .digit_1(d1),
        .digit_2(d2),
        .digit_3(d3),
        .clk(clk_17),
        .digit_sel(DIGIT),
        .display(DISPLAY)
    );

     // digits
    reg [3:0] digit_0; 
    reg [3:0] digit_1; 
    reg [3:0] digit_2; 
    reg [3:0] digit_3; 

    always @(*) 
    begin
        digit_0 = distance_0 % 10;
        digit_1 = distance_0 % 100 / 10;
        digit_2 = distance_1 % 10;
        digit_3 = distance_1 % 100 / 10;
    end

     // convert to 7 bit
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


endmodule