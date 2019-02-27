library verilog;
use verilog.vl_types.all;
entity brunit is
    port(
        idrd1           : in     vl_logic_vector(31 downto 0);
        idrd2           : in     vl_logic_vector(31 downto 0);
        branch          : in     vl_logic;
        bne             : in     vl_logic;
        pcsrc           : out    vl_logic
    );
end brunit;
