function []=plot_signals(signal_1, signal_2,tit, output_data_titel, x_label, y_label, tit_signal_1, tit_signal_2, num)
    figure;
    if num ==1
         m = 1:1:length(signal_1);
         plot(m, signal_1);
         tit_sig_1 = [tit_signal_1];
         tit_sig_2 = [];
         title(tit);
         legend(tit_sig_1);
        xlabel(x_label) % x-axis label
        ylabel(y_label) % y-axis label
        hold on;
        saveas(gcf,output_data_titel);
    elseif num ==2
        m = 1:1:length(signal_1);
        m2 = 1:1:length(signal_2);
        plot (m,signal_1,m2,signal_2);
        tit_sig_1 =  [tit_signal_1];
        tit_sig_2 = [tit_signal_2];
        title(tit);
        legend(tit_sig_1, tit_sig_2);
        xlabel(x_label) % x-axis label
        ylabel(y_label) % y-axis label
        hold on;
        saveas(gcf,output_data_titel);
    end