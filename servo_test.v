// 02 124 ~ 82
// 01 124 ~ 78

module servo_test(
    input clk,
    input rst, 
    input enable,
    input sw0,
    output PWM_0,
    output PWM_1,
    output [6:0] DISPLAY,
    output [3:0] DIGIT
);
    // clocks
    wire clk_17;
    clock_divider_17 clk_div_17_01(
        .clk(clk),
        .clk_div(clk_17)
    );

    // regs
    reg [9:0] duty_0;
    reg [9:0] duty_1;

    // generate pwm signal
    PWM_gen(
        .clk(clk),
        .reset(rst),
        .freq(32'd50),
        .duty(duty_0),
        .PWM(PWM_0)
    );

    // generate pwm signal
    PWM_gen(
        .clk(clk),
        .reset(rst),
        .freq(32'd50),
        .duty(duty_1),
        .PWM(PWM_1)
    );

    // change duty according to switch input
    always @(*) 
    begin
        if(enable)
        begin
            if(sw0)
            begin
                duty_0 = 79;
                duty_1 = 126;
            end
            else 
            begin
                duty_0 = 126;
                duty_1 = 82;
            end
        end 
        else
        begin
            duty_0 = 126;
            duty_1 = 126;
        end   
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
        digit_0 = duty_0 % 10;
        digit_1 = duty_0 % 100 / 10;
        digit_2 = duty_0 % 1000 / 100;
        digit_3 = duty_0 / 1000;
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