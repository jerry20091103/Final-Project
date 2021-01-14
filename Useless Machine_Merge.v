// uart8/Uart8Transmitter.v
/*
 * 8-bit UART Transmitter.
 * Able to transmit 8 bits of serial data, one start bit, one stop bit.
 * When transmit is complete {done} is driven high for one clock cycle.
 * When transmit is in progress {busy} is driven high.
 * Clock should be decreased to baud rate.
 */
// states of state machine
`define RESET       3'b001
`define IDLE        3'b010
`define START_BIT   3'b011 // transmitter only
`define DATA_BITS   3'b100
`define STOP_BIT    3'b101

module Uart8Transmitter (
    input  wire       clk,   // baud rate
    input  wire       en,
    input  wire       start, // start of transaction
    input  wire [7:0] in,    // data to transmit
    output reg        out,   // tx
    output reg        done,  // end on transaction
    output reg        busy   // transaction is in process
);
    reg [2:0] state  = `RESET;
    reg [7:0] data   = 8'b0; // to store a copy of input data
    reg [2:0] bitIdx = 3'b0; // for 8-bit data
    reg [2:0] idx;

    always@*
    begin
        idx = bitIdx;
    end

    always @(posedge clk) begin
        case (state)
            default     : begin
                state   <= `IDLE;
            end
            `IDLE       : begin
                out     <= 1'b1; // drive line high for idle
                done    <= 1'b0;
                busy    <= 1'b0;
                bitIdx  <= 3'b0;
                data    <= 8'b0;
                if (start & en) begin
                    data    <= in; // save a copy of input data
                    state   <= `START_BIT;
                end
            end
            `START_BIT  : begin
                out     <= 1'b0; // send start bit (low)
                busy    <= 1'b1;
                state   <= `DATA_BITS;
            end
            `DATA_BITS  : begin // Wait 8 clock cycles for data bits to be sent
                out     <= data[idx];
                if (&bitIdx) begin
                    bitIdx  <= 3'b0;
                    state   <= `STOP_BIT;
                end else begin
                    bitIdx  <= bitIdx + 1'b1;
                end
            end
            `STOP_BIT   : begin // Send out Stop bit (high)
                done    <= 1'b1;
                data    <= 8'b0;
                state   <= `IDLE;
            end
        endcase
    end

endmodule

// uart8/Uart8Receiver.v
/*
 * 8-bit UART Receiver.
 * Able to receive 8 bits of serial data, one start bit, one stop bit.
 * When receive is complete {done} is driven high for one clock cycle.
 * Output data should be taken away by a few clocks or can be lost.
 * When receive is in progress {busy} is driven high.
 * Clock should be decreased to baud rate.
 */
 // states of state machine
`define RESET       3'b001
`define IDLE        3'b010
`define START_BIT   3'b011 // transmitter only
`define DATA_BITS   3'b100
`define STOP_BIT    3'b101

module Uart8Receiver (
    input  wire       clk,  // baud rate
    input  wire       en,
    input  wire       in,   // rx
    output reg  [7:0] out,  // received data
    output reg        done, // end on transaction
    output reg        busy, // transaction is in process
    output reg        err   // error while receiving data
);
    // states of state machine
    reg [1:0] RESET = 2'b00;
    reg [1:0] IDLE = 2'b01;
    reg [1:0] DATA_BITS = 2'b10;
    reg [1:0] STOP_BIT = 2'b11;

    reg [2:0] state;
    reg [2:0] bitIdx = 3'b0; // for 8-bit data
    reg [1:0] inputSw = 2'b0; // shift reg for input signal state
    reg [3:0] clockCount = 4'b0; // count clocks for 16x oversample
    reg [7:0] receivedData = 8'b0; // temporary storage for input data

    initial begin
        out <= 8'b0;
        err <= 1'b0;
        done <= 1'b0;
        busy <= 1'b0;
    end

    always @(posedge clk) begin
        inputSw = { inputSw[0], in };

        if (!en) begin
            state = RESET;
        end

        case (state)
            RESET: begin
                out <= 8'b0;
                err <= 1'b0;
                done <= 1'b0;
                busy <= 1'b0;
                bitIdx <= 3'b0;
                clockCount <= 4'b0;
                receivedData <= 8'b0;
                if (en) begin
                    state <= IDLE;
                end
            end

            IDLE: begin
                done <= 1'b0;
                if (&clockCount) begin
                    state <= DATA_BITS;
                    out <= 8'b0;
                    bitIdx <= 3'b0;
                    clockCount <= 4'b0;
                    receivedData <= 8'b0;
                    busy <= 1'b1;
                    err <= 1'b0;
                end else if (!(&inputSw) || |clockCount) begin
                    // Check bit to make sure it's still low
                    if (&inputSw) begin
                        err <= 1'b1;
                        state <= RESET;
                    end
                    clockCount <= clockCount + 4'b1;
                end
            end

            // Wait 8 full cycles to receive serial data
            DATA_BITS: begin
                if (&clockCount) begin // save one bit of received data
                    clockCount <= 4'b0;
                    // TODO: check the most popular value
                    receivedData[bitIdx] <= inputSw[0];
                    if (&bitIdx) begin
                        bitIdx <= 3'b0;
                        state <= STOP_BIT;
                    end else begin
                        bitIdx <= bitIdx + 3'b1;
                    end
                end else begin
                    clockCount <= clockCount + 4'b1;
                end
            end

            /*
            * Baud clock may not be running at exactly the same rate as the
            * transmitter. Next start bit is allowed on at least half of stop bit.
            */
            STOP_BIT: begin
                if (&clockCount || (clockCount >= 4'h8 && !(|inputSw))) begin
                    state <= IDLE;
                    done <= 1'b1;
                    busy <= 1'b0;
                    out <= receivedData;
                    clockCount <= 4'b0;
                end else begin
                    clockCount <= clockCount + 1;
                    // Check bit to make sure it's still high
                    if (!(|inputSw)) begin
                        err <= 1'b1;
                        state <= RESET;
                    end
                end
            end

            default: state <= IDLE;
        endcase
    end

endmodule

// uart8/BaudRateGenerator.v
/*
 * Baud rate generator to divide {CLOCK_RATE} (internal board clock) into
 * a rx/tx {BAUD_RATE} pair with rx oversamples by 16x.
 */
module BaudRateGenerator  #(
    parameter CLOCK_RATE = 50000000, // board internal clock (def == 100MHz)
    parameter BAUD_RATE = 9600
)(
    input wire clk, // board clock
    output reg rxClk, // baud rate for rx
    output reg txClk // baud rate for tx
);
parameter MAX_RATE_RX = CLOCK_RATE / (2 * BAUD_RATE * 16); // 16x oversample
parameter MAX_RATE_TX = CLOCK_RATE / (2 * BAUD_RATE);
parameter RX_CNT_WIDTH = $clog2(MAX_RATE_RX);
parameter TX_CNT_WIDTH = $clog2(MAX_RATE_TX);

reg [RX_CNT_WIDTH - 1:0] rxCounter = 0;
reg [TX_CNT_WIDTH - 1:0] txCounter = 0;

initial begin
    rxClk = 1'b0;
    txClk = 1'b0;
end

always @(posedge clk) begin
    // rx clock
    if (rxCounter == MAX_RATE_RX[RX_CNT_WIDTH-1:0]) begin
        rxCounter <= 0;
        rxClk <= ~rxClk;
    end else begin
        rxCounter <= rxCounter + 1'b1;
    end
    // tx clock
    if (txCounter == MAX_RATE_TX[TX_CNT_WIDTH-1:0]) begin
        txCounter <= 0;
        txClk <= ~txClk;
    end else begin
        txCounter <= txCounter + 1'b1;
    end
end

endmodule

// sample_code/sonic.v
// sonic_top is the module to use sonic sensors
// clk = 100Mhz
// <Trig> and <Echo> should connect to the sensor
// <distance> is the output distance in cm
module sonic_top(clk, rst, Echo, Trig, distance);
	input clk, rst, Echo;
	output Trig;
    output [19:0] distance;

	wire[19:0] dis;
    wire clk1M;
	wire clk_2_17;

    assign distance = dis;

    div clk1(clk ,clk1M);
	TrigSignal u1(.clk(clk), .rst(rst), .trig(Trig));
	PosCounter u2(.clk(clk1M), .rst(rst), .echo(Echo), .distance_count(dis));
 
endmodule

module PosCounter(clk, rst, echo, distance_count); 
    input clk, rst, echo;
    output[19:0] distance_count;

    parameter S0 = 2'b00;
    parameter S1 = 2'b01; 
    parameter S2 = 2'b10;
    
    wire start, finish;
    reg[1:0] curr_state, next_state;
    reg echo_reg1, echo_reg2;
    reg[19:0] count, distance_register;
    wire[19:0] distance_count; 

    always@(posedge clk) begin
        if(rst) begin
            echo_reg1 <= 0;
            echo_reg2 <= 0;
            count <= 0;
            distance_register  <= 0;
            curr_state <= S0;
        end
        else begin
            echo_reg1 <= echo;   
            echo_reg2 <= echo_reg1; 
            case(curr_state)
                S0:begin
                    if (start) curr_state <= next_state; //S1
                    else count <= 0;
                end
                S1:begin
                    if (finish) curr_state <= next_state; //S2
                    else count <= count + 1;
                end
                S2:begin
                    distance_register <= count;
                    count <= 0;
                    curr_state <= next_state; //S0
                end
            endcase
        end
    end

    always @(*) begin
        case(curr_state)
            S0:next_state = S1;
            S1:next_state = S2;
            S2:next_state = S0;
            default:next_state = S0;
        endcase
    end

    assign distance_count = distance_register / 58; 
    assign start = echo_reg1 & ~echo_reg2;  
    assign finish = ~echo_reg1 & echo_reg2; 
endmodule

// send trigger signal to sensor
module TrigSignal(clk, rst, trig);
    input clk, rst;
    output trig;

    reg trig, next_trig;
    reg[23:0] count, next_count;

    always @(posedge clk, posedge rst) begin
        if (rst) begin
            count <= 0;
            trig <= 0;
        end
        else begin
            count <= next_count;
            trig <= next_trig;
        end
    end
    // count 10us
    always @(*) begin
        next_trig = trig;
        next_count = count + 1;
        if(count == 999)
            next_trig = 0;
        else if(count == 24'd9999999) begin
            next_trig = 1;
            next_count = 0;
        end
    end
endmodule

// clock divider for T = 1us clock
module div(clk ,out_clk);
    input clk;
    output out_clk;
    reg out_clk;
    reg [6:0]cnt;
    
    always @(posedge clk) begin   
        if(cnt < 7'd50) begin
            cnt <= cnt + 1'b1;
            out_clk <= 1'b1;
        end 
        else if(cnt < 7'd100) begin
	        cnt <= cnt + 1'b1;
	        out_clk <= 1'b0;
        end
        else if(cnt == 7'd100) begin
            cnt <= 0;
            out_clk <= 1'b1;
        end
    end
endmodule

// debounce onepulse.v
module debounce(pb_debounced, pb ,clk);
    output pb_debounced;
    input pb;
    input clk;
    
    reg [6:0] shift_reg;
    always @(posedge clk) begin
        shift_reg[6:1] <= shift_reg[5:0];
        shift_reg[0] <= pb;
    end
    
    assign pb_debounced = shift_reg == 7'b111_1111 ? 1'b1 : 1'b0;
endmodule

module onepulse(signal, clk, op);
    input signal, clk;
    output op;
    
    reg op;
    reg delay;
    
    always @(posedge clk) begin
        if((signal == 1) & (delay == 0)) op <= 1;
        else op <= 0; 
        delay = signal;
    end
endmodule

// pwn_gen.v
//generte PWM by input frequency & duty
module PWM_gen (
    input wire clk,
    input wire reset,
	input [31:0] freq,
    input [9:0] duty,
    output reg PWM
);
    wire [31:0] count_max = 50_000_000 / freq;
    wire [31:0] count_duty = count_max * duty / 1024;
    reg [31:0] count;
        
    always @(posedge clk, posedge reset) begin
        if (reset) begin
            count <= 0;
            PWM <= 0;
        end else if (count < count_max) begin
            count <= count + 1;
            if(count < count_duty)
                PWM <= 1;
            else
                PWM <= 0;
        end else begin
            count <= 0;
            PWM <= 0;
        end
    end
endmodule

// display_driver.v
// A led 7_segment display control module
// clk should be equal to refresh rate
// clk: 100 MHz / 2**17
module led_display(
    input clk,
    input [6:0] digit_0,
    input [6:0] digit_1,
    input [6:0] digit_2,
    input [6:0] digit_3,
    output reg [3:0] digit_sel,
    output reg [6:0] display
    );

    reg [1:0] count;

    // increase count by one
    always @(posedge clk)
    begin
        if(count == 2'b11)
        begin
            count = 2'b00;
        end
        else
        begin
            count = count + 1;
        end
    end

    // Select Digits
    always @*
    begin
        case(count)
            0: digit_sel = 4'b1110;
            1: digit_sel = 4'b1101;
            2: digit_sel = 4'b1011;
            3: digit_sel = 4'b0111;
            default : digit_sel = 4'b1111;
        endcase
    end
    // Select input digit
    always @* 
    begin
        case(count)
            0: display = digit_0;
            1: display = digit_1;
            2: display = digit_2;
            3: display = digit_3;
            default: display = 7'b1111111;
        endcase
    end
    

endmodule

// convert int 1~9 to 7-bit code
module int_to_7_bit(
    input [3:0] digit_in,
    output reg [6:0] digit_out
);
    always @*
    begin
        case (digit_in)
            0: digit_out = 7'b1000000;
            1: digit_out = 7'b1111001;
            2: digit_out = 7'b0100100;
            3: digit_out = 7'b0110000;
            4: digit_out = 7'b0011001;
            5: digit_out = 7'b0010010;
            6: digit_out = 7'b0000010;
            7: digit_out = 7'b1111000;
            8: digit_out = 7'b0000000;
            9: digit_out = 7'b0010000;
            10: digit_out = 7'b0111111; // "----"
            11: digit_out = 7'b0000110; // E
            12: digit_out = 7'b0101111; // r
            default: digit_out = 7'b1111111;
        endcase
    end

endmodule

// motor_control.v
// A module to spin wheels
// enable 0 = stop, 1 = run
// dir 1 = cw (back) , 0 = ccw (forward)
module motor_control(
    input l_enable,
    input l_dir,
    input r_enable,
    input r_dir,
    output reg [1:0] motor_cw,
    output reg [1:0] motor_ccw
);
    // left motor
    always @(*) 
    begin
        if(l_enable)
        begin
            if(l_dir)
            begin
                motor_cw[0] = 1;
                motor_ccw[0] = 0;
            end
            else
            begin
                motor_cw[0] = 0;
                motor_ccw[0] = 1;
            end
        end 
        else 
        begin
            motor_cw[0] = 0;
            motor_ccw[0] = 0;
        end   
    end
    // right motor
    always @(*) 
    begin
        if(r_enable)
        begin
            if(r_dir)
            begin
                motor_cw[1] = 1;
                motor_ccw[1] = 0;
            end
            else
            begin
                motor_cw[1] = 0;
                motor_ccw[1] = 1;
            end
        end 
        else 
        begin
            motor_cw[1] = 0;
            motor_ccw[1] = 0;
        end   
    end

endmodule

// servo_control.v
// A module to control fliping arms
// clk = 100Mhz
// only allowed to control one arm at a time to prevent danger
// if both arms are up at the same time, they may crash into each other
module servo_control(
    input clk,
    input rst, 
    input enable,       // enalbe = 0 -> both arms hide
    input select,       // select which servo to control, 0 or 1
    input [4:0] amount, // 0~31, 0 = hide, 31 = hit the switch
    output PWM_0,       // outputs to servo pwm ports
    output PWM_1
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

    // change duty according to amount and select
    // 1 126 ~ 82
    // 0 126 ~ 78
    // in  0 ~ 31
    always @(*) 
    begin
        if(enable)
        begin
            // servo 0
            if(~select)
            begin
                duty_1 = 126;
                case (amount)
                    0:  duty_0 = 126;
                    1:  duty_0 = 124;
                    2:  duty_0 = 123;
                    3:  duty_0 = 121;
                    4:  duty_0 = 120;
                    5:  duty_0 = 118;
                    6:  duty_0 = 117;
                    7:  duty_0 = 115;
                    8:  duty_0 = 114;
                    9:  duty_0 = 112;
                    10:  duty_0 = 111;
                    11:  duty_0 = 109;
                    12:  duty_0 = 107;
                    13:  duty_0 = 106;
                    14:  duty_0 = 104;
                    15:  duty_0 = 103;
                    16:  duty_0 = 101;
                    17:  duty_0 = 100;
                    18:  duty_0 = 98;
                    19:  duty_0 = 97;
                    20:  duty_0 = 95;
                    21:  duty_0 = 93;
                    22:  duty_0 = 92;
                    23:  duty_0 = 90;
                    24:  duty_0 = 89;
                    25:  duty_0 = 87;
                    26:  duty_0 = 86;
                    27:  duty_0 = 84;
                    28:  duty_0 = 83;
                    29:  duty_0 = 81;
                    30:  duty_0 = 80;
                    31:  duty_0 = 78;
                    default: duty_0 = 126;
                endcase
            end 
            // servo 1
            else
            begin
                duty_0 = 126;
                case (amount)
                    0:  duty_1 = 126;
                    1:  duty_1 = 124;
                    2:  duty_1 = 123;
                    3:  duty_1 = 121;
                    4:  duty_1 = 120;
                    5:  duty_1 = 119;
                    6:  duty_1 = 117;
                    7:  duty_1 = 116;
                    8:  duty_1 = 115;
                    9:  duty_1 = 113;
                    10:  duty_1 = 112;
                    11:  duty_1 = 110;
                    12:  duty_1 = 109;
                    13:  duty_1 = 108;
                    14:  duty_1 = 106;
                    15:  duty_1 = 105;
                    16:  duty_1 = 103;
                    17:  duty_1 = 102;
                    18:  duty_1 = 100;
                    19:  duty_1 = 99;
                    20:  duty_1 = 98;
                    21:  duty_1 = 96;
                    22:  duty_1 = 95;
                    23:  duty_1 = 93;
                    24:  duty_1 = 92;
                    25:  duty_1 = 91;
                    26:  duty_1 = 89;
                    27:  duty_1 = 88;
                    28:  duty_1 = 86;
                    29:  duty_1 = 85;
                    30:  duty_1 = 84;
                    31:  duty_1 = 82;
                    default: duty_1 = 126;
                endcase
            end
        end
        else
        begin
            duty_0 = 126;
            duty_1 = 126;
        end
    end
endmodule

// bluetooth_control.v
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

// lfsr_random.v
// a pseudo random number generator based on lfsr
// output 2 bit random number
// clk = the generating frequency
// a different number every clock cycle
module lfsr_random(
    input clk,
    input rst,
    output [1:0] random
);

    reg [3:0] state;
    always @(posedge clk, posedge rst) 
    begin
        if(rst)
            state <= 4'b1000;
        else
        begin
            state[2:0] <= state[3:1];
            state[3] <= state[1] ^ state[0];
        end
    end

    assign random[0] = state[0] ^ state[1];
    assign random[1] = state[2] ^ state[3];

endmodule

// Useless Machine.v
// main modules of useless machine.
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
  input ble_rx,            // bluetooth chip
  output ble_tx
);

//__Basic Parameter__//
parameter clk_basic = 17;
parameter clk_led = 22;

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
reg[15:0] LED_next;
wire[1:0] random_lfsr;
reg sw0_final, sw0_final_next;
// reg[1:0] random, random_next;
// reg servo_sel, servo_enable;

wire [6:0] d0;
wire [6:0] d1;
wire [6:0] d2;
wire [6:0] d3;

reg [3:0] digit_0; 
reg [3:0] digit_1; 
reg [3:0] digit_2; 
reg [3:0] digit_3; 

//__External Signal__//
//__Servo__//
reg servo_enable;
reg servo_sel;
reg [4:0] servo_amount;

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
wire [5:0] command;
wire ble_err;

// __Basic Device__ //
clock_divider #(.n(clk_basic)) clk_div_17 (
    .clk(clk_100), 
    .clk_div(clk_17)
);

clock_divider #(.n(clk_led)) clk_div_22 (
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

//------------------------------//
// __Big State Machine__ //
// __Mode__ //
parameter NS = 2'b00;  // Normal Switch.
parameter UB = 2'b01;  // Useless Box.
parameter UC = 2'b10;  // Useful Car.

reg[1:0] state, state_next;

always@(posedge clk or posedge rst) begin
    if(rst) begin
        state = NS;
    end else if((mode_p) || (command[5])) begin
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

//__Random Control__//
//__Random State Machine__//
// For good_dis and delta, see "servo control" below.
// Dodge time = Dodge seconds / 0.04, where 0.04 = 2**22 / 100MHz
`define good_dis 20 
`define delta 4      
`define dodge_time 75

parameter RANDOM = 2'b00;
parameter CLASSIC = 2'b01;
parameter DODGE = 2'b10;
parameter ADVANCE = 2'b11;

parameter INIT = 1'b0;
parameter MOVE = 1'b1;

reg[1:0] state_random, state_random_next;
reg state_sw0, state_sw0_next;
reg[7:0] dodge_counter, dodge_counter_next;

always@(posedge clk or posedge rst) begin
    if(rst) begin
        state_random = RANDOM;
    end else begin
        state_random = state_random_next;
    end
end

always@(*) begin
    if(state != UB) begin
        state_random_next = RANDOM;
    end else if(wanted_ub == 3'b000) begin
        if(state_random == RANDOM) begin
            if(random_lfsr == 0) begin
                state_random_next = ADVANCE;
            end else if(random_lfsr == 1) begin
                state_random_next = CLASSIC;
            end else if(random_lfsr == 2) begin
                state_random_next = DODGE;
            end else begin
                state_random_next = RANDOM;
            end
        end else if(state_random == CLASSIC) begin
            if ((state_sw0 == MOVE) && (sw0 == sw0_final)) begin
                state_random_next = RANDOM;
            end else begin
                state_random_next = CLASSIC;
            end
        end else if(state_random == DODGE) begin
            if(dodge_counter == `dodge_time) begin
                state_random_next = RANDOM;
            end else begin
                state_random_next = DODGE;
            end
        end else begin
            if ((state_sw0 == MOVE) && (sw0 == sw0_final)) begin
                state_random_next = RANDOM;
            end else begin
                state_random_next = ADVANCE;
            end
        end
    end else if(wanted_ub == 3'b001) begin
        state_random_next = CLASSIC;
    end else if(wanted_ub == 3'b010) begin
        state_random_next = DODGE;
    end else if(wanted_ub == 3'b100) begin
        state_random_next = ADVANCE;
    end else begin
        state_random_next = RANDOM;
    end
end

//__Sw0 State Machine__//
always@(posedge clk or posedge rst) begin
    if(rst) begin
        state_sw0 = INIT;
    end else begin
        state_sw0 = state_sw0_next;
    end
end

always@(*) begin
    if(state_random == RANDOM) begin
        if(state == UB) begin
            state_sw0_next = INIT;
        end else begin
            if(state_sw0 == INIT) begin
                if(command[0]) begin
                    state_sw0_next = MOVE;
                end else if(sw0 != sw0_final) begin
                    state_sw0_next = MOVE;
                end else begin
                    state_sw0_next = INIT;
                end
            end else if(sw0 != sw0_final) begin
                state_sw0_next = MOVE;
            end else begin
                state_sw0_next = INIT;
            end
        end
    end else if(state_random == CLASSIC) begin
        state_sw0_next = (sw0 == sw0_final) ? INIT : MOVE;
    end else if(state_random == DODGE) begin
        if(state_sw0 == INIT) begin
            if((distance_0 < `good_dis) || (distance_1 + `delta < `good_dis)) begin
                state_sw0_next = MOVE;
            end else begin
                state_sw0_next = INIT;
            end
        end else begin
            state_sw0_next = (dodge_counter == `dodge_time) ? INIT : MOVE;
        end
    end else begin
        state_sw0_next = (sw0 == sw0_final) ? INIT : MOVE;
    end
end

//__Dodge Counter__//
always@(posedge clk_22 or posedge rst) begin
    if(rst) begin
        dodge_counter = 8'd0;
    end else begin
        dodge_counter = dodge_counter_next;
    end
end

always@(*) begin
    if((state_random == DODGE) && (state_sw0 == MOVE)) begin
        dodge_counter_next = dodge_counter + 1'd1;
    end else begin
        dodge_counter_next = 8'd0;
    end
end

always@(*) begin
    if(state == NS) begin
        // led will shine if sw0 is on
        if(sw0) begin
            LED_next[15:0] = 16'b1111_1111_1111_1111;
        end else begin
            LED_next[15:0] = 16'b0000_0000_0000_0000;
        end
    end else if(state == UB) begin
       // show wanted ub
        if(wanted_ub != 3'b000) begin
            if(state_random == ADVANCE) begin
                LED_next[2:0] = 3'b100;
            end else if(state_random == CLASSIC) begin
                LED_next[2:0] = 3'b001;
            end else if(state_random == DODGE) begin
                LED_next[2:0] = 3'b010;
            end else begin
                LED_next[2:0] = wanted_ub;
            end
        end else begin
            LED_next[2:0] = wanted_ub;
        end
       LED_next[15:3] = 13'd0;
    end else begin
        if(command[3]) begin
            // left
            if(LED == 16'b0000_0000_0000_0001) begin
                LED_next[15:0] = 16'b1000_0000_0000_0000;
            end else begin
                LED_next[15:0] = LED[15:0] >> 1'b1;
            end
        end else if(command[4]) begin
            // right
            if(LED == 16'b1000_0000_0000_0000) begin
                LED_next[15:0] = 16'b0000_0000_0000_0001;
            end else begin
                LED_next[15:0] = LED[15:0] << 1'b1;
            end
        end else begin
            // forward and backward
            LED_next[15:8] = 8'd0;
            LED_next[7] = 1'b1;
            LED_next[6:0] = 7'd0;
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

//__External Device__//
//__Random (LFSR)__//
lfsr_random lfsr(
    .clk(clk),
    .rst(rst),
    .random(random_lfsr)
);

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
    .l_dir(~motor_l_dir),
    .r_dir(~motor_r_dir),
    .motor_cw(motor_cw),
    .motor_ccw(motor_ccw)
);

//__Bluetooth__//
bluetooth_control ble_ctrl_m(
    .clk(clk),
    .rst(rst),
    .ble_rx(ble_rx),
    .ble_tx(ble_tx),
    .ble_err(ble_err),
    .cur_state(state),
    .switch(command[0]),
    .forward(command[1]),
    .backward(command[2]),
    .left(command[3]),
    .right(command[4]),
    .mode(command[5])
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
        sw0_final = 0;
    end else begin
        sw0_final = sw0_final_next;
    end
end

always@(*) begin
    if(state != UB) begin
        if(state_sw0 == INIT) begin
            if(command[0]) begin
                sw0_final_next = ~sw0_final;
            end else begin
                sw0_final_next = (state == NS) ? sw0 : sw0_final; 
            end
        end else begin
            sw0_final_next = sw0_final;
        end
    end else begin
        sw0_final_next = sw0_final;
    end
end
//__Servo Control__//
// NOTICE:
// 1. good_dis is the distance when user isn't near the box.
//    Usually it should be the length of the box, but you can deine your own good_dis as well.
//    In that case, this "closer effect" may not be suitable.
// 2. delta = left - right. (left is longer)
//    This is because our box is not symmetric. If it is symmetric, it should be 0.
// 3. For more detailed information, check "closer effect" section below.
always@(*)begin
    if(rst) begin
        servo_enable = 0;
        servo_sel = ~sw0;
        servo_amount = 0;
    end else begin
        if(state == NS) begin
            if((state_sw0 == MOVE) && (!ir_sensor_deb)) begin
                servo_enable = 1;
                servo_sel = ~sw0;
                servo_amount = 31;
            end else begin
                servo_enable = 0;
                servo_sel = ~sw0;
                servo_amount = 0;
            end
        end else if(state == UB) begin
            if(state_random == CLASSIC) begin
                if((state_sw0 == MOVE) && (!ir_sensor_deb)) begin
                    servo_enable = 1;
                    servo_sel = ~sw0;
                    servo_amount = 31;
                end else begin
                    servo_enable = 0;
                    servo_sel = ~sw0;
                    servo_amount = 0;
                end
            end else if(state_random == DODGE) begin
                servo_enable = 0;
                servo_sel = ~sw0;
                servo_amount = 0;
            end else if(state_random == ADVANCE) begin
                servo_enable = 1;
                servo_sel = ~sw0;
                if(state_sw0 == INIT) begin
                    if((distance_0 > `good_dis) && (distance_1 + `delta > `good_dis)) begin
                        servo_amount = 0;
                    end else if(ir_sensor_deb) begin
                        servo_amount = 31; 
                    end else if(distance_0 > distance_1 + `delta) begin
                        servo_amount = ((distance_1 + `delta) * 3 < 25) ? (distance_1 + `delta) * 3 : 31;
                    end else begin
                        servo_amount = (distance_0 * 3 < 25) ? distance_0 * 3 : 31;
                    end
                end else begin
                    if(!ir_sensor_deb) begin
                        servo_amount = 31;
                    end else begin
                        servo_amount = 0;
                    end
                end
            end else begin
                servo_enable = 0;
                servo_sel = ~sw0;
                servo_amount = 0;
            end
        end else begin
            if((state_sw0 == MOVE) && (!ir_sensor_deb)) begin
                servo_enable = 1;
                servo_sel = ~sw0;
                servo_amount = 31;
            end else begin
                servo_enable = 0;
                servo_sel = ~sw0;
                servo_amount = 0;
            end
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
        if((state_random == DODGE) && (state_sw0 == MOVE) && (dodge_counter < `dodge_time)) begin
            if((distance_0 > `good_dis) && (distance_1 + `delta > `good_dis)) begin
                motor_l_enable = 0;
                motor_r_enable = 0;
                motor_l_dir = 0;
                motor_r_dir = 0;
            end else if(distance_0 > distance_1 + `delta) begin
                motor_l_enable = 1;
                motor_r_enable = 1;
                motor_l_dir = 0;
                motor_r_dir = 0;
            end else begin
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