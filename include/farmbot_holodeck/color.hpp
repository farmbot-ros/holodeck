#include <cstdint>
#include <rerun.hpp>
#include <stdexcept>
#include <string>

namespace holodeck {

    /**
     * @brief Converts a hexadecimal color string to a rerun::Color object.
     *
     * This function accepts hex strings in the formats:
     * - #RRGGBB or #RRGGBBAA for full notation.
     * - #RGB or #RGBA for shorthand notation, which are expanded to full form.
     *
     * @param hex A string representing the hex color (e.g., "#FF5733" or "#F53").
     * @return rerun::Color The corresponding color object with red, green, blue, and alpha components.
     *
     * @throws std::invalid_argument if the hex string is empty, does not start with '#',
     *         or does not have 3, 4, 6, or 8 hex digits after the '#' character.
     */
    inline rerun::Color hexToColor(const std::string &hex) {
        // Validate hex string
        if (hex.empty() || hex[0] != '#') {
            throw std::invalid_argument("Invalid hex color format. Expected format: #RRGGBB or #RGB");
        }

        std::string hexColor = hex.substr(1);
        size_t length = hexColor.size();

        if (length != 3 && length != 4 && length != 6 && length != 8) {
            throw std::invalid_argument("Invalid hex color length. Expected 3, 4, 6, or 8 hex digits.");
        }

        // Expand shorthand hex notation (#RGB or #RGBA) to full form (#RRGGBB or #RRGGBBAA)
        if (length == 3 || length == 4) {
            std::string expanded;
            for (char c : hexColor) {
                expanded.push_back(c);
                expanded.push_back(c);
            }
            hexColor = expanded;
            length = hexColor.size();
        }

        // Parse the hex color components.
        uint8_t r = std::stoul(hexColor.substr(0, 2), nullptr, 16);
        uint8_t g = std::stoul(hexColor.substr(2, 2), nullptr, 16);
        uint8_t b = std::stoul(hexColor.substr(4, 2), nullptr, 16);
        uint8_t a = (length == 8) ? std::stoul(hexColor.substr(6, 2), nullptr, 16) : 255;

        return rerun::Color(r, g, b, a);
    }

    /**
     * @brief Adjusts the brightness of a hexadecimal color.
     *
     * This function modifies the brightness of a given hex color string by darkening or lightening it.
     * The adjustment parameter must be in the range [-1, 1]:
     * - A positive value (closer to 1) will darken the color by reducing each RGB channel.
     * - A negative value (closer to -1) will lighten the color by increasing each RGB channel towards 255.
     *
     * @param hex A string representing the hex color (e.g., "#FF5733" or "#F53").
     * @param adjustment A float in the range [-1, 1] indicating the brightness adjustment.
     *                   Values > 0 darken the color, values < 0 lighten the color.
     * @return rerun::Color The adjusted color after applying the brightness modification.
     *
     * @note The alpha channel remains unchanged.
     */
    inline rerun::Color adjustColor(const std::string &hex, float adjustment) {
        // Clamp adjustment between -1 and 1.
        if (adjustment > 1.0f)
            adjustment = 1.0f;
        if (adjustment < -1.0f)
            adjustment = -1.0f;

        // Convert the hex string to a Color.
        rerun::Color color = hexToColor(hex);

        // Retrieve the original RGBA components.
        uint8_t r = color.r();
        uint8_t g = color.g();
        uint8_t b = color.b();
        uint8_t a = color.a();

        if (adjustment >= 0) {
            // Darken the color: reduce each channel by a fraction of its current value.
            r = static_cast<uint8_t>(r * (1.0f - adjustment));
            g = static_cast<uint8_t>(g * (1.0f - adjustment));
            b = static_cast<uint8_t>(b * (1.0f - adjustment));
        } else {
            // Lighten the color: increase each channel towards 255 by a fraction.
            float factor = -adjustment;
            r = static_cast<uint8_t>(r + (255 - r) * factor);
            g = static_cast<uint8_t>(g + (255 - g) * factor);
            b = static_cast<uint8_t>(b + (255 - b) * factor);
        }

        return rerun::Color(r, g, b, a);
    }

} // namespace holodeck
