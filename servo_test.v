module servo_test(
    input clk,
    input rst, 
    input btn_up,
    input btn_down,
    output PWM_0,
    output [6:0] DISPLAY,
    output [3:0] DIGIT
);
    // clocks
    wire clk_17;
    clock_divider_17 clk_div_17_01(
        .clk(clk),
        .clk_div(clk_17)
    );
    // debounce and one pulse
    wire btn_up_deb, btn_up_op;
    wire btn_down_deb, btn_down_op;
    debounce btn_up_deb_m(
        .pb(btn_up),
        .pb_debounced(btn_up_deb),
        .clk(clk_17)
    );
    debounce btn_down_deb_m(
        .pb(btn_down),
        .pb_debounced(btn_down_deb),
        .clk(clk_17)
    );
    onepulse btn_up_op_m(
        .signal(btn_up_deb),
        .clk(clk),
        .op(btn_up_op)
    );
    onepulse btn_down_op_m(
        .signal(btn_down_deb),
        .clk(clk),
        .op(btn_down_op)
    );

    // regs
    reg [9:0] duty;
    reg [9:0] duty_next;

    // generate pwm signal
    PWM_gen(
        .clk(clk),
        .reset(rst),
        .freq(32'd50),
        .duty(duty),
        .PWM(PWM_0)
    );

    // control servo with pushbuttons in 10 steps
    always @(posedge clk, posedge rst)
    begin
        if(rst)
            duty = 26;
        else
            duty = duty_next; 
    end
    always @*
    begin
        if(btn_up_op)
        begin
            if(duty < 126)
                duty_next = duty + 10;
            else
                duty_next = 126;
        end
        else if(btn_down_op)
        begin
            if(duty > 26)
                duty_next = duty - 10;
            else
                duty_next = 26;
        end
        else
            duty_next = duty;
    end

    // show duty value on display
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
        digit_0 = duty % 10;
        digit_1 = duty % 100 / 10;
        digit_2 = duty % 1000 / 100;
        digit_3 = duty / 1000;
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