library verilog;
use verilog.vl_types.all;
entity idfwunit is
    port(
        idRs            : in     vl_logic_vector(4 downto 0);
        idRt            : in     vl_logic_vector(4 downto 0);
        wbRd            : in     vl_logic_vector(4 downto 0);
        wbregwrite      : in     vl_logic;
        idfwA           : out    vl_logic;
        idfwB           : out    vl_logic
    );
end idfwunit;
