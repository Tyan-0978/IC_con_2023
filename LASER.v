// -----------------------------------------------------------------------------
// 2023 IC contest group E
// -----------------------------------------------------------------------------

module LASER (
    input        CLK,
    input        RST,
    input  [3:0] X,
    input  [3:0] Y,
    output [3:0] C1X,
    output [3:0] C1Y,
    output [3:0] C2X,
    output [3:0] C2Y,
    output DONE
);

// -----------------------------------------------------------------------------
// parameters & variables
// -----------------------------------------------------------------------------
integer i;
genvar j;

// FSM
parameter S_IN  = 0;
parameter S_TRV = 1;
parameter S_NEW = 2;
parameter S_END = 3;

// traverse state
parameter TS_R = 0;
parameter TS_L = 1;
parameter TS_D = 2;
parameter TS_CALC = 3;
parameter TS_END  = 4;

// -----------------------------------------------------------------------------
// signal declaration
// -----------------------------------------------------------------------------
// FSM
reg  [1:0] state, next_state;
reg  [2:0] trv_state, next_trv_state;

// control signals
wire converge;
wire calc_end, trv_end;
reg  [5:0] in_count, next_in_count;
reg  [1:0] cvg_count, next_cvg_count;

// cover points
reg  [3:0] points_x [0:39], next_points_x [0:39];
reg  [3:0] points_y [0:39], next_points_y [0:39];
reg  is_valid [0:39], next_is_valid [0:39];

// traverse
reg  [3:0] x, next_x;
reg  [3:0] y, next_y;

reg  [5:0] cover_num, next_cover_num;
wire [3:0] calc_tmp;
reg  [2:0] calc_count, next_calc_count;

reg  [5:0] max_cover, next_max_cover;
reg  [3:0] max_x, next_max_x;
reg  [3:0] max_y, next_max_y;

// cover sub-module
wire cv_out [0:7];
reg  valid_cv_out [0:7];
wire [3:0] cv_cx, cv_cy;

// circles
reg  [3:0] cx1, next_cx1;
reg  [3:0] cy1, next_cy1;
reg  [3:0] cx2, next_cx2;
reg  [3:0] cy2, next_cy2;

// -----------------------------------------------------------------------------
// outputs
// -----------------------------------------------------------------------------
assign C1X = cx1;
assign C1Y = cy1;
assign C2X = cx2;
assign C2Y = cy2;
assign DONE = (state == S_END);

// -----------------------------------------------------------------------------
// sub-modules
// -----------------------------------------------------------------------------
assign cv_cx = (state == S_TRV) ? x : max_x;
assign cv_cy = (state == S_TRV) ? y : max_y;

generate
    for (j = 0; j < 8; j = j + 1) begin
        is_covered cv0 (
            .ci(cv_cx), .cj(cv_cy), 
            .pi(points_x[j]), .pj(points_y[j]), 
            .covered(cv_out[j])
            );
    end
endgenerate

// -----------------------------------------------------------------------------
// FSM
// -----------------------------------------------------------------------------
always @(*) begin
    // next state
    case (state)
        S_IN : next_state = (in_count == 39) ? S_TRV : S_IN;
        S_TRV: next_state = (trv_end) ? S_NEW : S_TRV;
        S_NEW: begin
            if (calc_end) begin
                next_state = (converge && cvg_count >= 2) ? S_END : S_TRV;
            end
            else begin
                next_state = S_NEW;
            end
        end
        S_END: next_state = S_IN;

        default: next_state = S_IN;
    endcase

    // next traverse state
    if (state == S_TRV) begin
        case (trv_state)
            TS_R: next_trv_state = TS_CALC;
            TS_L: next_trv_state = TS_CALC;
            TS_D: next_trv_state = TS_CALC;
            TS_CALC: next_trv_state = (calc_end) ? TS_END : TS_CALC;
            TS_END: begin
                if (y[0]) begin  // odd y
                    if (x == 0) next_trv_state = TS_D;
                    else        next_trv_state = TS_L;
                end
                else begin  // even y
                    if (x == 15) next_trv_state = TS_D;
                    else         next_trv_state = TS_R;
                end
            end

            default: next_trv_state = TS_CALC;
        endcase
    end
    else begin
        next_trv_state = TS_CALC;
    end
end

always @(posedge CLK or posedge RST) begin
    if (RST) begin
        state <= S_IN;
        trv_state <= TS_CALC;
    end
    else begin
        state <= next_state;
        trv_state <= next_trv_state;
    end
end

// -----------------------------------------------------------------------------
// combinational part
// -----------------------------------------------------------------------------
// control signals
assign converge = (state == S_NEW && max_x == cx1 && max_y == cy1);
assign calc_end = (calc_count == 4);
assign trv_end = (x == 0 && y == 15 && trv_state == TS_END);
always @(*) begin
    if (state == S_IN) begin
        next_in_count = in_count + 1;
    end
    else begin
        next_in_count = 0;
    end

    if (trv_end && cvg_count < 2) begin
        next_cvg_count = cvg_count + 1;
    end
    else begin
        next_cvg_count = cvg_count;
    end
end

// cover points
always @(*) begin
    case (state)
        S_IN: begin
            next_points_x[0] = X;
            next_points_y[0] = Y;
            for (i = 1; i < 40; i = i + 1) begin
                next_points_x[i] = points_x[i-1];
                next_points_y[i] = points_y[i-1];
            end
            for (i = 0; i < 40; i = i + 1) begin
                next_is_valid[i] = 1;
            end
        end
        S_TRV: begin
            if (trv_state == TS_CALC) begin
                for (i = 0; i < 8; i = i + 1) begin
                    next_points_x[i] = points_x[i+32];
                    next_points_y[i] = points_y[i+32];
                    next_is_valid[i] = is_valid[i+32];
                end
                for (i = 8; i < 40; i = i + 1) begin
                    next_points_x[i] = points_x[i-8];
                    next_points_y[i] = points_y[i-8];
                    next_is_valid[i] = is_valid[i-8];
                end
            end
            else begin
                for (i = 0; i < 40; i = i + 1) begin
                    next_points_x[i] = points_x[i];
                    next_points_y[i] = points_y[i];
                    next_is_valid[i] = is_valid[i];
                end
            end
        end
        S_NEW: begin
            for (i = 0; i < 8; i = i + 1) begin
                next_points_x[i] = points_x[i+32];
                next_points_y[i] = points_y[i+32];
            end
            for (i = 8; i < 40; i = i + 1) begin
                next_points_x[i] = points_x[i-8];
                next_points_y[i] = points_y[i-8];
            end

            // update is_valid; set covered points to unvalid
            for (i = 0; i < 8; i = i + 1) begin
                next_is_valid[i] = is_valid[i+32];
            end
            for (i = 8; i < 16; i = i + 1) begin
                next_is_valid[i] = ~cv_out[i-8];
            end
            for (i = 16; i < 40; i = i + 1) begin
                next_is_valid[i] = is_valid[i-8];
            end
        end

        default: begin
            for (i = 0; i < 40; i = i + 1) begin
                next_points_x[i] = points_x[i];
                next_points_y[i] = points_y[i];
                next_is_valid[i] = is_valid[i];
            end
        end
    endcase
end

// traverse
always @(*) begin
    for (i = 0; i < 8; i = i + 1) begin
        valid_cv_out[i] = cv_out[i] & is_valid[i];
    end
end
assign calc_tmp = valid_cv_out[0] + valid_cv_out[1] 
                + valid_cv_out[2] + valid_cv_out[3] 
                + valid_cv_out[4] + valid_cv_out[5] 
                + valid_cv_out[6] + valid_cv_out[7];
                //+ valid_cv_out[8] + valid_cv_out[9];

always @(*) begin
    // indices
    if (state == S_TRV) begin
        case (trv_state)
            TS_R: begin
                next_x = x + 1;
                next_y = y;
            end
            TS_L: begin
                next_x = x - 1;
                next_y = y;
            end
            TS_D: begin
                next_x = x;
                next_y = y + 1;
            end

            default: begin
                next_x = x;
                next_y = y;
            end
        endcase
    end
    else begin
        next_x = 0;
        next_y = 0;
    end

    // calculation count
    if (state == S_TRV) begin
        case (trv_state)
            TS_CALC: next_calc_count = calc_count + 1;
            TS_END : next_calc_count = 0;

            default: next_calc_count = calc_count;
        endcase
    end
    else if (state == S_NEW) begin
        next_calc_count = calc_count + 1;
    end
    else begin
        next_calc_count = 0;
    end

    // number of covered points
    if (state == S_TRV) begin
        case (trv_state)
            TS_CALC: next_cover_num = cover_num + calc_tmp;
            TS_END : next_cover_num = 0;

            default: next_cover_num = cover_num;
        endcase
    end
    else begin
        next_cover_num = 0;
    end

    // current max
    case (state)
        S_TRV: begin
            if (trv_state == TS_END) begin
                next_max_cover = (max_cover > cover_num) ? max_cover : cover_num;
                next_max_x = (max_cover > cover_num) ? max_x : x;
                next_max_y = (max_cover > cover_num) ? max_y : y;
            end
            else begin
                next_max_cover = max_cover;
                next_max_x = max_x;
                next_max_y = max_y;
            end
        end
        S_NEW: begin
            if (calc_end) begin
                next_max_cover = 0;
                next_max_x = 0;
                next_max_y = 0;
            end
            else begin
                next_max_cover = max_cover;
                next_max_x = max_x;
                next_max_y = max_y;
            end
        end

        default: begin
            next_max_cover = max_cover;
            next_max_x = max_x;
            next_max_y = max_y;
        end
    endcase
end

// circles
always @(*) begin
    case (state)
        S_NEW: begin
            if (calc_end) begin
                next_cx1 = cx2;
                next_cy1 = cy2;
                next_cx2 = max_x;
                next_cy2 = max_y;
            end
            else begin
                next_cx1 = cx1;
                next_cy1 = cy1;
                next_cx2 = cx2;
                next_cy2 = cy2;
            end
        end

        default: begin
            next_cx1 = cx1;
            next_cy1 = cy1;
            next_cx2 = cx2;
            next_cy2 = cy2;
        end
    endcase
end

// -----------------------------------------------------------------------------
// sequential part
// -----------------------------------------------------------------------------
always @(posedge CLK or posedge RST) begin
    if (RST) begin
        in_count <= 0;
        cvg_count <= 0;
        for (i = 0; i < 40; i = i + 1) begin
            points_x[i] <= 0;
            points_y[i] <= 0;
            is_valid[i] <= 0;
        end
        x <= 0; y <= 0;
        cover_num <= 0;
        calc_count <= 0;
        max_cover <= 0;
        max_x <= 0; max_y <= 0;
        cx1 <= 0; cy1 <= 0;
        cx2 <= 0; cy2 <= 0;
    end
    else begin
        in_count <= next_in_count;
        cvg_count <= next_cvg_count;
        for (i = 0; i < 40; i = i + 1) begin
            points_x[i] <= next_points_x[i];
            points_y[i] <= next_points_y[i];
            is_valid[i] <= next_is_valid[i];
        end
        x <= next_x; y <= next_y;
        cover_num <= next_cover_num;
        calc_count <= next_calc_count;
        max_cover <= next_max_cover;
        max_x <= next_max_x; max_y <= next_max_y;
        cx1 <= next_cx1; cy1 <= next_cy1;
        cx2 <= next_cx2; cy2 <= next_cy2;
    end
end

endmodule


module is_covered (
    input  [3:0] ci,
    input  [3:0] cj,
    input  [3:0] pi,
    input  [3:0] pj,
    output covered
);

wire [3:0] diff_i, diff_j;
wire [3:0] valid;

assign covered = |valid;

assign diff_i = (ci > pi) ? (ci - pi) : (pi - ci);
assign diff_j = (cj > pj) ? (cj - pj) : (pj - cj);

assign valid[0] = (diff_i == 0) & (diff_j <= 4);
assign valid[1] = (diff_i <= 2) & (diff_j <= 3);
assign valid[2] = (diff_i <= 3) & (diff_j <= 2);
assign valid[3] = (diff_i <= 4) & (diff_j == 0);

endmodule
