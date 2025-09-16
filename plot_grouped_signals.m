function plot_grouped_signals(signals, labels, iterationNames)
    % signals: either a long vector or matrix [N_signals x signal_length]
    % labels:  cell array of strings (e.g., {'Start', 'Mid', 'End'}) [must be 1×N]
    % iterationNames: optional cell array for legend (e.g., {'Trial A', 'Trial B'})

    % Convert to matrix if vector is passed
    signals = vector_to_signal_matrix(signals, 24); % hardcoded 24 as your known label length

    % Make sure labels are row vector
    labels = reshape(labels, 1, []);

    % Basic dimensions
    n_labels = numel(labels);
    signal_length = size(signals, 2);

    if mod(signal_length, n_labels) ~= 0
        warning('Signal length is not a multiple of number of labels. Truncating extra samples.');
        max_len = floor(signal_length / n_labels) * n_labels;
        signals = signals(:, 1:max_len);
        signal_length = max_len;
    end

    % Repeat labels to cover full signal length
    repeated_labels = repmat(labels, 1, signal_length / n_labels);
    x = categorical(repeated_labels);

    % Set iteration names if not given
    n_signals = size(signals, 1);
    if nargin < 3 || isempty(iterationNames)
        iterationNames = arrayfun(@(i) sprintf('Iteration %d', i), 1:n_signals, 'UniformOutput', false);
    end
    
    % ✅ Ensure it's a cell array of char or string
    if isstring(iterationNames)
        iterationNames = cellstr(iterationNames);
    elseif isnumeric(iterationNames)
        iterationNames = arrayfun(@(x) num2str(x), iterationNames, 'UniformOutput', false);
    end


    % Plot
    figure; hold on;
    colors = lines(n_signals);
    for i = 1:n_signals
        scatter(x, signals(i, :), 50, 'filled', ...
            'MarkerFaceColor', colors(i,:), ...
            'DisplayName', iterationNames{i});
    end

    xlabel('Labels');
    ylabel('Signal Value');
    title('Grouped Signal Scatter Plot');
    legend('show');
    grid on;
end
