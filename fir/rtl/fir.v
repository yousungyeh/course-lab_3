`timescale 1ns / 1ps

module fir 
#(  parameter pADDR_WIDTH = 12,
    parameter pDATA_WIDTH = 32,
    parameter Tape_Num    = 11,
    parameter Addr_offset = 2,
    // state
    parameter IDLE = 3'b000,
    parameter RADDR = 3'b001,
    parameter RDATA = 3'b010,
    parameter WADDR = 3'b011,
    parameter WDATA = 3'b100,
    
    parameter fir_IDLE = 2'b00,
    parameter fir_PROG = 2'b01,
    parameter fir_COMP = 2'b10,
    parameter fir_DONE = 2'b11
)
(
    // axi4-lite write transaction
    input   wire [(pADDR_WIDTH-1):0] awaddr,
    input   wire                     awvalid,
    output  wire                     awready,
    
    input   wire                     wvalid,
    output  wire                     wready,
    input   wire [(pDATA_WIDTH-1):0] wdata, 
    // axi4-lite read transaction
    input   wire [(pADDR_WIDTH-1):0] araddr,
    input   wire                     arvalid,
    output  wire                     arready,
    
    output  wire                     rvalid,    
    input   wire                     rready,
    output  wire [(pDATA_WIDTH-1):0] rdata,
    
    //axi4-stream slave
    input   wire                     ss_tvalid, 
    output  wire                     ss_tready, 
    input   wire [(pDATA_WIDTH-1):0] ss_tdata, 
    input   wire                     ss_tlast, 
    //axi4-stream master
    output  wire                     sm_tvalid, 
    input   wire                     sm_tready, 
    output  wire [(pDATA_WIDTH-1):0] sm_tdata, 
    output  wire                     sm_tlast, 
    
    // bram for tap RAM
    output  wire [3:0]               tap_WE,
    output  wire                     tap_EN,
    output  wire [(pDATA_WIDTH-1):0] tap_Di,
    output  wire [(pADDR_WIDTH-1):0] tap_A,
    input   wire [(pDATA_WIDTH-1):0] tap_Do,

    // bram for data RAM
    output  wire [3:0]               data_WE,
    output  wire                     data_EN,
    output  wire [(pDATA_WIDTH-1):0] data_Di,
    output  wire [(pADDR_WIDTH-1):0] data_A,
    input   wire [(pDATA_WIDTH-1):0] data_Do,

    input   wire                     axis_clk,
    input   wire                     axis_rst_n
);
begin

    /**********************************************/
    /*           define register                  */
    /**********************************************/
    // Configuration Register Address map
    reg [(pDATA_WIDTH-1):0] ap_ctrl_reg, data_length_reg;
    
    // FSM state register
    reg [2:0] state, next_state; // AXI_Lite FSM
    reg [1:0] fir_state, fir_next_state; // FIR FSM
    
    // temp sum
    reg signed [(pDATA_WIDTH-1):0] sum; 
    
    /**********************************************/
    /*           define register END              */
    /**********************************************/
    
    /**********************************************/
    /*           AXI_Lite Program                 */
    /**********************************************/
    // AR channel
    assign arready = (state == RADDR)? 1 : 0;
    
    // R channel
    assign rdata   = (state == RDATA)?
        (araddr == 32'h0000)? ap_ctrl_reg : 
        (araddr == 32'h0010)? data_length_reg : 
        (araddr >= 32'h0020)? tap_Do : 0 : 0 ;
    assign rvalid  = (state == RDATA)? 1 : 0;
    
    // AW channel
    assign awready = (state == WADDR)? 1 : 0;
    
    // W channel
    assign wready  = (state == WDATA)? 1 : 0;
  
    // next state for AXI-Lite
    always @(*) begin
		case (state)
			IDLE  : next_state = (arvalid)? RADDR : (awvalid)? WADDR : IDLE;
			RADDR : next_state = (arvalid && arready)? RDATA : RADDR;
			RDATA : next_state = (rvalid  && rready )? IDLE  : RDATA;
			WADDR : next_state = (awvalid && awready)? WDATA : WADDR;
			WDATA : next_state = (wvalid  && wready )? IDLE  : WDATA;
			default : next_state = IDLE;
		endcase
	end
	always @(posedge axis_clk) begin
        if (~axis_rst_n) begin
            state <= IDLE;
        end else begin
            state <= next_state;
        end
    end
    
    // ap_ctrl & data_length register configuration 
    integer i;
    always @(posedge axis_clk) begin
        if (~axis_rst_n) begin
            ap_ctrl_reg[0] <= 0; // ap_start
            ap_ctrl_reg[1] <= 0; // ap_done
            ap_ctrl_reg[2] <= 1; // ap_idle
            for (i = 3; i < pDATA_WIDTH; i=i+1) begin
                ap_ctrl_reg[i] <= 0;
            end
            data_length_reg <= 0;
        end 
        else begin
            if (state == WDATA) begin
                if (awaddr == 32'h0000) begin // 0x00
                    ap_ctrl_reg <= wdata;
                end 
                else if (awaddr == 32'h0010) begin // 0x10
                    data_length_reg <= wdata;
                end
            end 
            else if (state == RDATA) begin // When ap_done is read, i.e. address 0x00 is read
                ap_ctrl_reg[1] <= (awaddr == 32'h0000)? 0 : ap_ctrl_reg[1];
            end 
            else begin
                // ap_start
                ap_ctrl_reg[0] <= (ap_ctrl_reg[0] == 0)? 0 : 
                                  (fir_state == fir_IDLE)? 1 : 0;
                // ap_done is asserted when engine completes last data processing and data is transferred
                ap_ctrl_reg[1] <= (ap_ctrl_reg[1] == 1)? 1 :
                                  (sm_tlast == 1 && fir_state == fir_DONE)? 1 : 0;
                // ap_idle
                ap_ctrl_reg[2] <= (ap_ctrl_reg[2] == 1)? ((ap_ctrl_reg[0] == 1)? 0 : 1) :
                                  (fir_state == fir_IDLE)? 1 : 0;
            end      
        end
    end
    /**********************************************/
    /*              AXI_Lite Program END          */
    /**********************************************/
       
    /**********************************************/
    /*              AXI_Stream                    */
    /**********************************************/  
    
    reg [3:0] cnt;
    reg [3:0] write_ptr;
    wire [3:0] read_ptr;
    // flag for last fir output
    reg last; 
    
    assign ss_tready = (fir_state == fir_PROG)? 1 : 0;
    assign sm_tvalid = (cnt == Tape_Num)? 1 : 0;
    assign sm_tdata = sum;
    assign sm_tlast  = (fir_state == fir_DONE)? ss_tlast : 0;
    
    // read pointer and write pointer
    assign read_ptr = (write_ptr >= cnt)? (write_ptr-cnt) : Tape_Num-(cnt-write_ptr);
    
    // computation
    always @(posedge axis_clk) begin
        if (~axis_rst_n) begin
            write_ptr <= 0;
            cnt <= 0;
            sum <= 0;
            last <= 0;
		end 
        else begin 
            case (fir_state)
                fir_IDLE:begin
                    write_ptr <= (write_ptr == 10)? 0 : write_ptr+1;
                end
                fir_PROG:begin
                    cnt <= 1;
                end
                fir_COMP:begin
                    cnt <= (cnt == Tape_Num)? 0 : cnt+1;
                    sum <= sum + $signed(tap_Do) * $signed(data_Do);                    // 1 Multiplier & 1 adder
                end
                fir_DONE:begin                 
                    write_ptr <= (write_ptr == 10)? 0 : write_ptr+1;
                    sum <= 0;
                    last <= (ss_tlast == 1)? 1:0;
                end
            endcase
        end
    end
    
    // next state for AXI-Stream
    always @(*) begin
        case (fir_state)
            fir_IDLE: fir_next_state = (ap_ctrl_reg[0] == 1)? fir_PROG : fir_IDLE;    // wait ap_start to start fir engine       
            fir_PROG: fir_next_state = fir_COMP;            
            fir_COMP: fir_next_state = (cnt == Tape_Num)? fir_DONE : fir_COMP;
            fir_DONE: fir_next_state = (last == 1)? fir_IDLE : fir_PROG;
            default:  fir_next_state = fir_IDLE;
        endcase
    end
    always @(posedge axis_clk) begin
        if(~axis_rst_n)begin
            fir_state <= fir_IDLE;
        end
        else begin
            fir_state <= fir_next_state;
        end
    end
    
    /**********************************************/
    /*              AXI_Stream END                */
    /**********************************************/
    
    /**********************************************/
    /*           tap BRAM (coefficient)           */
    /**********************************************/
    
    assign tap_EN = 1;
    assign tap_WE = (state == WDATA && awaddr >= 32'h0020)? 4'b1111:0; // only AXI_lite write coefficient
    assign tap_Di = wdata;
    assign tap_A  = (state == WDATA && fir_state == fir_IDLE)? (awaddr-32'h0020) : // write coefficient
                   (state == RADDR && fir_state == fir_IDLE)? (araddr-32'h0020) :  // read coefficient
                   (cnt << Addr_offset);
                   
    /**********************************************/
    /*           tap BRAM (coefficient) END       */
    /**********************************************/

    /**********************************************/
    /*           data BRAM (Xn)                   */
    /**********************************************/
    
    assign data_EN = 1;
    assign data_WE = (fir_state == fir_IDLE || fir_state == fir_PROG)? 4'b1111 : 0;
    assign data_Di = (fir_state == fir_IDLE)? 0 : ss_tdata;
    assign data_A  = (fir_state == fir_COMP)? (read_ptr << Addr_offset) : (write_ptr << Addr_offset);

    /**********************************************/
    /*           data BRAM (Xn) END               */
    /**********************************************/

end
endmodule