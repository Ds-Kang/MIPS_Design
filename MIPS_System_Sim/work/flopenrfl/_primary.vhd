library verilog;
use verilog.vl_types.all;
entity flopenrfl is
    port(
        clk             : in     vl_logic;
        reset           : in     vl_logic;
        en              : in     vl_logic;
        fl              : in     vl_logic;
        d               : in     vl_logic_vector(32 downto 0);
        q               : out    vl_logic_vector(32 downto 0)
    );
end flopenrfl;
