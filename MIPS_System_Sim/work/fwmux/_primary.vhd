library verilog;
use verilog.vl_types.all;
entity fwmux is
    port(
        fw              : in     vl_logic_vector(1 downto 0);
        a               : in     vl_logic_vector(31 downto 0);
        b               : in     vl_logic_vector(31 downto 0);
        c               : in     vl_logic_vector(31 downto 0);
        y               : out    vl_logic_vector(31 downto 0)
    );
end fwmux;
