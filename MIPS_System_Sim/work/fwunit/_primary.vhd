library verilog;
use verilog.vl_types.all;
entity fwunit is
    port(
        exRs            : in     vl_logic_vector(4 downto 0);
        exRt            : in     vl_logic_vector(4 downto 0);
        memRd           : in     vl_logic_vector(4 downto 0);
        wbRd            : in     vl_logic_vector(4 downto 0);
        memregwrite     : in     vl_logic;
        wbregwrite      : in     vl_logic;
        fwA             : out    vl_logic_vector(1 downto 0);
        fwB             : out    vl_logic_vector(1 downto 0)
    );
end fwunit;
