module EG_LOGIC_IDDR (q1, q0, clk, d, rst);
    output q1;        //  [Template:1 bit negedge data output.]
    output q0;        //  [Template:1 bit posedge data output.]
    input clk;        //  [Template:Sampling clock.]
    input d;          //  [Template:DDR data entered on I/O.]
    input rst;        //  [Template:Reset signal,high active.]

    parameter ASYNCRST = "ENABLE";    //  "ENABLE", "DISABLE". [Template:Asynchronous reset.]
    parameter PIPEMODE = "PIPED";     //  "PIPED", "NONE". [Template:q0,q1 data alignment mode.]

    tri1 gsr_n = glbl.gsrn;
    wire rst_node = (ASYNCRST=="ENABLE")? rst:1'b0;
    wire async_rst = ~gsr_n | rst_node;
    wire clk_node = clk; 
    reg [1:0] comp_reg,true_reg;

    
    always @(negedge clk_node or posedge async_rst)
    begin
     if(async_rst)
     comp_reg[0] <= 1'b0 ;
     else
     comp_reg[0] <= d ;
    end
    always @(posedge clk_node or posedge async_rst)
    begin
     if(async_rst)
     comp_reg[1] = 1'b0 ;
     else
     comp_reg[1] <= comp_reg[0] ;
    end
    
    always @(posedge clk_node or posedge async_rst)
    begin
     if(async_rst)
     true_reg[0] <= 1'b0 ;
     else
     true_reg[0] <= d ;
    end
    always @(posedge clk_node or posedge async_rst)
    begin
     if(async_rst)
     true_reg[1] = 1'b0 ;
     else
     true_reg[1] <= true_reg[0] ;
    end    
    
    wire true_dff = (PIPEMODE == "PIPED")? true_reg[1]:true_reg[0];
    buf (q1, comp_reg[1]);
    buf (q0, true_dff);          	

endmodule
module EG_LOGIC_ODDR (q, clk, d1, d0, rst);
    output q;         //  [Template:1 bit DDR edge output data.]
    input clk;        //  [Template:Synchronous clock.]
    input d1;         //  [Template:1 bit negedge input data.]
    input d0;         //  [Template:1 bit posedge input data.]
    input rst;        //  [Template:Reset,high active.]
    
    parameter ASYNCRST = "ENABLE";    //  "ENABLE", "DISABLE". [Template:Asynchronous reset.]

    tri1 gsr_n = glbl.gsrn;
    wire rst_node = (ASYNCRST=="ENABLE")? rst:1'b0;
    wire async_rst = ~gsr_n | rst_node ;   
    wire clk_node = clk;
    reg [1:0] comp_reg;
    reg true_reg;   
    wire ddr;
    buf (d_comp,d1);
    buf (d_true,d0);
    buf (q,ddr);
    always @(posedge clk_node or posedge async_rst)
    begin
     if(async_rst)
     comp_reg[0] = 1'b0 ;
     else
     comp_reg[0] = d_comp ;
    end
    always @(negedge clk_node or posedge async_rst)
    begin
     if(async_rst)
     comp_reg[1] = 1'b0 ;
     else
     comp_reg[1] = comp_reg[0] ;
    end

    always @(posedge clk_node or posedge async_rst)
    begin
     if(async_rst)
     true_reg = 1'b0 ;
     else
     true_reg = d_true ;
    end

    reg clk_mux;
    always @(clk_node)  clk_mux <= #0.1 clk_node; 
    assign   ddr = clk_mux ? true_reg : comp_reg[1] ;    

endmodule
module glbl; wire gsrn=1; endmodule
module EG_LOGIC_BUFG(output o, input i); assign o=i; endmodule
module EG_PHY_PLL(clkc,extlock,stdby,refclk,fbclk,reset,load_reg,psclk,psdown,psstep,psclksel,psdone,dclk,dcs,dwe,di,daddr,do);
 output [4:0] clkc; output extlock,psdone; output [7:0] do; input stdby,refclk,fbclk,reset,load_reg,psclk,psdown,psstep,dclk,dcs,dwe; input [2:0] psclksel; input [7:0] di; input [5:0] daddr;
 parameter DPHASE_SOURCE="",DYNCFG="",FIN="",FEEDBK_MODE="",FEEDBK_PATH="",STDBY_ENABLE="",PLLRST_ENA="",SYNC_ENABLE="";
 parameter GMC_GAIN=0,GMC_TEST=0,ICP_CURRENT=0,KVCO=0,LPF_CAPACITOR=0,LPF_RESISTOR=0,REFCLK_DIV=1,FBCLK_DIV=1;
 parameter CLKC0_ENABLE="",CLKC0_DIV=1,CLKC0_CPHASE=0,CLKC0_FPHASE=0,CLKC1_ENABLE="",CLKC1_DIV=1,CLKC1_CPHASE=0,CLKC1_FPHASE=0;
 assign clkc=0; assign extlock=0; assign psdone=0; assign do=0;
endmodule
