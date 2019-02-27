library verilog;
use verilog.vl_types.all;
entity aludec is
    port(
        funct           : in     vl_logic_vector(5 downto 0);
        aluop           : in     vl_logic_vector(1 downto 0);
        slti            : in     vl_logic;
        jr              : out    vl_logic;
        alucontrol      : out    vl_logic_vector(2 downto 0)
    );
end aludec;
