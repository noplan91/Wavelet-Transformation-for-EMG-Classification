 function [art_all, con_all] = read_in_data( art_all, con_all, step, maximum)

time=1/2000:1/2000:64000/2000;

%read in contractions
begin='1_ext_strong';
for i=1:1:21
    file=[begin mat2str(i)];
    data1=load(file);
    data2=data1.src1.Data';

    contraction=data2(step:step:100000);
    
    %Remove Offset
    contraction=(contraction-mean(contraction));
    
    con_all=[con_all; contraction];
end

art_all = [];
%read in artefacts
begin='1_ext_art';
for i=1:1:81
    file=[begin mat2str(i)];
    data1=load(file);
    data2=data1.src1.Data';

    artefact=data2(step:step:100000);
    
    %Remove Offset
    artefact=(artefact-mean(artefact));
    
    art_all=[art_all; artefact];
end

%normalize to 1024
art_all =( maximum*art_all/max(art_all));
con_all =( maximum*con_all/max(con_all));


% cut all signals that are smaller than 2% of the maximal signal value
% get maxima of the signals
max_art = 0.02*max(art_all);
max_con = 0.02*max(con_all);

% cut signals down
art_all = art_all(abs(art_all) >= max_art);
con_all = con_all(abs(con_all) >= max_con);

% floor 
art_all = (art_all);
con_all = (con_all);

