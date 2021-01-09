// A bluetooth remote control module
// clk = 100Mhz
module bluetooth_control(
    input clk,
    input rst, 
    input ble_rx,        // connect to output of the hc-05 bluetooth chip
    output ble_err,      // indicate transmission error for debug purpose
    // user commands, 1 = on, 0 = off
    // they all work independently, ex forward = 1 and backward = 1 at the same time
    output reg forward,  // move car forward
    output reg backward, //          backward
    output reg left,     //          left
    output reg right,    //          right
    output reg switch    // flip the switch
);

    // clocks
    wire rx_clk;
    BaudRateGenerator brg_0(
        .clk(clk),
        .rxClk(rx_clk)
    );

    // uart interface
    wire [7:0] rx_data;
    wire ble_valid;

    Uart8Receiver uart_rx_m(
        .clk(rx_clk),
        .en(1),
        .in(ble_rx),
        .out(rx_data),
        .done(ble_valid),
        .err(ble_err)
    );

    // regs
    reg forward_next;
    reg backward_next;
    reg left_next;
    reg right_next;
    reg switch_0;
    reg switch_1;
    reg data_valid;

    // check if valid
    always @(*) 
    begin
        if(ble_valid && !ble_err && rx_data[7] == 1 && rx_data[6] == 1)
            data_valid = 1;
        else
            data_valid = 0;
    end

    // convert switch to one pulse
    always @(*) 
    begin
        if(switch_0 ^ switch_1)
            switch <= 1;
        else
            switch <= 0;
    end
    always @(posedge clk, posedge rst) 
    begin
        if(rst)
        begin
            switch_0 <= 0;
            switch_1 <= 1;
        end
        else
        begin
            switch_1 <= switch_0;
            switch_0 <= rx_data[0];
        end
    end

    always @(posedge clk, posedge rst)
    begin
        if(rst)
        begin
            forward = 0;
            backward = 0;
            left = 0;
            right = 0;
        end
        else
        begin
            forward = forward_next;
            backward = backward_next;
            left = left_next;
            right = right_next;
        end
    end
    always @(*) 
    begin
        if(data_valid)
        begin
            forward_next = rx_data[1];
            backward_next = rx_data[2];
            left_next = rx_data[3];
            right_next = rx_data[4];
        end
        else
        begin
            forward_next = forward;
            backward_next = backward;
            left_next = left;
            right_next = right;
        end
    end

endmodule