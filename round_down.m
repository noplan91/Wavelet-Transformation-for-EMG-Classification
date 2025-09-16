function [out] = round_down(in)

    if (in <= 0)
        in= in+1;
        out = floor(in);
    else
        out = floor(in);
    end
end

