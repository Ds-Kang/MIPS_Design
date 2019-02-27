library verilog;
use verilog.vl_types.all;
entity hdunit is
    port(
        exRt            : in     vl_logic_vector(4 downto 0);
        idRs            : in     vl_logic_vector(4 downto 0);
        idRt            : in     vl_logic_vector(4 downto 0);
        exmemread       : in     vl_logic;
        conts           : out    vl_logic;
        pcwrite         : out    vl_logic;
        idwrite         : out    vl_logic
    );
end hdunit;
