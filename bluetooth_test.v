module bluetooth_test(
    input clk,
    input rst, 
    input ble_rx,
    output [6:0] DISPLAY,
    output [3:0] DIGIT,
    output led_err,
    output led_busy,
    output [7:0] led

);
    // clocks
    wire clk_17;
    wire rx_clk;
    clock_divider_17 clk_17_m(
        .clk(clk),
        .clk_div(clk_17)
    );
    BaudRateGenerator brg_0(
        .clk(clk),
        .rxClk(rx_clk)
    );

    wire data_valid;
    wire [7:0] rx_data;
    wire rx_busy;
    wire rx_err;

    Uart8Receiver uart_rx_m(
        .clk(rx_clk),
        .en(1),
        .in(ble_rx),
        .out(rx_data),
        .done(data_valid),
        .busy(rx_busy),
        .err(rx_err)
    );

    assign led_err = rx_err;
    assign led_busy = led_busy;

    assign led[0] = rx_data[0];
    assign led[1] = rx_data[1];
    assign led[2] = rx_data[2];
    assign led[3] = rx_data[3];
    assign led[4] = rx_data[4];
    assign led[5] = rx_data[5];
    assign led[6] = rx_data[6];
    assign led[7] = rx_data[7];

    // show message on display
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
    reg [3:0] digit_0_n; 
    reg [3:0] digit_1_n; 
    reg [3:0] digit_2_n; 
    reg [3:0] digit_3_n; 

    always @(posedge clk, posedge rst)
    begin
        if(rst)
        begin
            digit_0 = 0;
            digit_1 = 0;
            digit_2 = 0;
            digit_3 = 0;
        end
        else
        begin
            digit_0 = digit_0_n;
            digit_1 = digit_1_n;
            digit_2 = digit_2_n;
            digit_3 = digit_3_n;
        end
    end

    always @*
    begin
        if(data_valid)
        begin
            digit_0_n = rx_data % 10;
            digit_1_n = rx_data % 100 / 10;
            digit_2_n = rx_data % 1000 / 100;
            digit_3_n = rx_data % 10000 / 1000;
        end
        else 
        begin
            digit_0_n = digit_0;
            digit_1_n = digit_1;
            digit_2_n = digit_2;
            digit_3_n = digit_3;
        end
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