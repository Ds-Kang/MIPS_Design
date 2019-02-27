library verilog;
use verilog.vl_types.all;
entity datapath is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        signext         : in     vl_logic;
        shiftl16        : in     vl_logic;
        memtoreg        : in     vl_logic;
        pcsrc           : in     vl_logic;
        memread         : in     vl_logic;
        memread2        : in     vl_logic;
        alusrc          : in     vl_logic;
        regdst          : in     vl_logic;
        regwrite        : in     vl_logic;
        regwrite2       : in     vl_logic;
        jump            : in     vl_logic;
        jal             : in     vl_logic;
        jr              : in     vl_logic;
        alucontrol      : in     vl_logic_vector(2 downto 0);
        zero            : out    vl_logic;
        conts           : out    vl_logic;
        pc              : out    vl_logic_vector(31 downto 0);
        instr2          : out    vl_logic_vector(31 downto 0);
        instr3          : out    vl_logic_vector(31 downto 0);
        instr           : in     vl_logic_vector(31 downto 0);
        aluout2         : out    vl_logic_vector(31 downto 0);
        writedata3      : out    vl_logic_vector(31 downto 0);
        readdata        : in     vl_logic_vector(31 downto 0)
    );
end datapath;
