// A bluetooth remote control module
// clk = 50Mhz
module bluetooth_control(
    input clk,
    input rst, 
    input ble_rx,        // connect to output of the hc-05 bluetooth chip
    input [1:0] cur_state, // current state of the box
    output ble_tx,       // connect to rx of hc05
    output ble_err,      // indicate transmission error for debug purpose
    // user commands, 1 = on, 0 = off
    // they all work independently, ex forward = 1 and backward = 1 at the same time
    output reg forward,  // move car forward
    output reg backward, //          backward
    output reg left,     //          left
    output reg right,    //          right
    output reg switch,   // flip the switch
    output reg mode      // switch modes
);

    // clocks
    wire rx_clk;
    wire tx_clk;
    wire clk_23;
    BaudRateGenerator brg_0(
        .clk(clk),
        .rxClk(rx_clk),
        .txClk(tx_clk)
    );
    clock_divider #(.n(23)) clk_div_23 (
        .clk(clk), 
        .clk_div(clk_23)
    );
    onepulse start_op(
        .clk(tx_clk),
        .signal(clk_23),
        .op(start_tx)
    ); // use this to send every 0.1 sec

    // uart interface
    wire [7:0] rx_data;
    wire ble_valid;
    wire [7:0] tx_data;
    wire tx_busy;
    wire tx_send;
    assign tx_send = start_tx && (!tx_busy);


    Uart8Receiver uart_rx_m(
        .clk(rx_clk),
        .en(1),
        .in(ble_rx),
        .out(rx_data),
        .done(ble_valid),
        .err(ble_err)
    );

    Uart8Transmitter uart_tx_m(
        .clk(tx_clk),
        .en(1),
        .start(tx_send),
        .in(tx_data),
        .out(ble_tx),
        .busy(tx_busy)
    );

    // regs
    reg forward_next;
    reg backward_next;
    reg left_next;
    reg right_next;
    reg switch_0;
    reg switch_1;
    reg mode_0;
    reg mode_1;
    reg data_valid;
    reg switch_data, switch_data_next;
    reg mode_data, mode_data_next;

    always @(posedge clk, posedge rst) 
    begin
        if(rst)
        begin
            switch_data <= 0;
            mode_data <= 0;
        end
        else
        begin
            mode_data <= mode_data_next;
            switch_data <= switch_data_next;
        end
    end
    always @(*) 
    begin
       if(data_valid)
       begin
            switch_data_next = rx_data[0];
            mode_data_next = rx_data[5];
       end
       else 
       begin
            switch_data_next = switch_data;
            mode_data_next = mode_data;
       end
    end

    assign tx_data = {6'b000000, cur_state};

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
            switch_1 <= 0;
        end
        else
        begin
            switch_1 <= switch_0;
            switch_0 <= switch_data;
        end
    end

    // convert mode to one pulse
    always @(*) 
    begin
        if(mode_0 ^ mode_1)
            mode <= 1;
        else
            mode <= 0;
    end
    always @(posedge clk, posedge rst) 
    begin
        if(rst)
        begin
            mode_0 <= 0;
            mode_1 <= 0;
        end
        else
        begin
            mode_1 <= mode_0;
            mode_0 <= mode_data;
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