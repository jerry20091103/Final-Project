module clock_divider_17(
    clk,
    clk_div
    );
    parameter n = 17;
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

module sonic_sensor_test(
    input clk,
    input rst,
    output trig,
    input echo,
    output [6:0] DISPLAY,
    output [3:0] DIGIT
);

    // clocks
    wire clk_17;
    clock_divider_17 clk_17_m(
        .clk(clk),
        .clk_div(clk_17)
    );

    wire [19:0] distance;

    sonic_top sonic_01(
        .clk(clk),
        .rst(rst),
        .Echo(echo),
        .Trig(trig),
        .distance(distance)
    );
    
    // wire up led display
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
        digit_0 = distance % 10;
        digit_1 = distance % 100 / 10;
        digit_2 = distance % 1000 / 100;
        digit_3 = 0;
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