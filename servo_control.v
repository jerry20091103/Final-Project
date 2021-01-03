// A module to control fliping arms
// clk = 100Mhz
module servo_control(
    input clk,
    input rst, 
    input enable,       // enalbe = 0 -> both arms hide
    input select,       // select which servo to control, 0 or 1
    input [4:0] amount, // 0~31, 0 = hide, 31 = hit the switch
    output PWM_0,       // output to servo pwm ports
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