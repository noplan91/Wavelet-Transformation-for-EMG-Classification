function signals = vector_to_signal_matrix(vec, signal_length)
    % vec: 1D input vector
    % signal_length: number of columns per row in output matrix
    
    vec = vec(:)';  % ensure it's a row vector
    total_length = length(vec);

    % Compute number of full signals that fit
    n_signals = floor(total_length / signal_length);

    % Truncate to exact multiple of signal_length
    vec = vec(1 : n_signals * signal_length);

    % Reshape into matrix [n_signals x signal_length]
    signals = reshape(vec, signal_length, n_signals)';
end
