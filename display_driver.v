// A led 7_segment display control module
// clk should be equal to refresh rate
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
            default: digit_out = 7'b1111111;
        endcase
    end

endmodule