// A module to spin wheels
// enable 0 = stop, 1 = run
// dir 1 = cw, 0 = ccw
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