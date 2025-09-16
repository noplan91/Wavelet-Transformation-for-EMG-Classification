function plot_signal_with_labels(signal, xLabels, xLabelName)
    x = 1:length(signal);
    figure;
    plot(x, signal, '-o');
    set(gca, 'XTick', x, 'XTickLabel', xLabels);
    xlabel(xLabelName);
    ylabel('Threshold');
    title("WPT Threshold vs. "+xLabelName);
    grid on;
    % Save in folder 'plots_2505' with the xlabel as filename
    if ~exist('plots_2505', 'dir')
        mkdir('plots_2505');
    end
    saveas(gcf, fullfile('plots_2505', [xLabelName, '.png']));
end