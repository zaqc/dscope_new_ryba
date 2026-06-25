// =========================================================================
// Auxiliary Module: CDC Safe Synchronizer Stage (ascan_sync)
// =========================================================================

module ascan_sync #(
    parameter WIDTH = 1
)(
    input  wire             clk,
    input  wire             rst,
    input  wire [WIDTH-1:0] i_sig,
    output wire [WIDTH-1:0] o_sig
);

    reg [WIDTH-1:0] sync_reg_0;
    reg [WIDTH-1:0] sync_reg_1;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            sync_reg_0 <= {WIDTH{1'b0}};
            sync_reg_1 <= {WIDTH{1'b0}};
        end else begin
            sync_reg_0 <= i_sig;
            sync_reg_1 <= sync_reg_0;
        end
    end

    assign o_sig = sync_reg_1;

endmodule
