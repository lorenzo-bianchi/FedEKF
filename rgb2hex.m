function hex_colors = rgb2hex(rgb_colors)
    % Initialize an empty character array to store the hex color strings
    hex_colors = strings(1, size(rgb_colors, 1));
    for idx = 1:size(rgb_colors, 1)
        % Ensure the RGB values are within the range [0, 1]
        rgb = min(max(rgb_colors(idx, :), 0), 1);
    
        % Scale to [0, 255] and round to the nearest integer
        rgb = round(rgb * 255);
    
        % Convert to hexadecimal and concatenate
        hex_colors(idx) = sprintf('#%02X%02X%02X', rgb);
    end
end
