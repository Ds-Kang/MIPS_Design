library verilog;
use verilog.vl_types.all;
entity controller is
    port(
        op              : in     vl_logic_vector(5 downto 0);
        funct           : in     vl_logic_vector(5 downto 0);
        zero            : in     vl_logic;
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        conts           : in     vl_logic;
        signext         : out    vl_logic;
        shiftl16        : out    vl_logic;
        memtoreg4       : out    vl_logic;
        memwrite3       : out    vl_logic;
        memread3        : out    vl_logic;
        memread2        : out    vl_logic;
        pcsrc           : out    vl_logic;
        alusrc2         : out    vl_logic;
        regdst2         : out    vl_logic;
        regwrite4       : out    vl_logic;
        regwrite3       : out    vl_logic;
        jump2           : out    vl_logic;
        jal2            : out    vl_logic;
        jr              : out    vl_logic;
        alucontrol      : out    vl_logic_vector(2 downto 0)
    );
end controller;
