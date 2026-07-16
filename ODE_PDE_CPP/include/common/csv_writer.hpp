#pragma once

#include <fstream>
#include <string>
#include <vector>
#include <stdexcept>

// This header provides a minimal CSV writer.
// The goal is to keep the code very explicit and heavily commented.

namespace common {

inline void write_csv(
    const std::string& filename,
    const std::vector<std::string>& header,
    const std::vector<std::vector<double>>& columns
)
{
    // Basic validation: we need at least one column.
    if (columns.empty())
        throw std::runtime_error("write_csv: no columns provided");

    // All columns must have the same length (same number of rows).
    const std::size_t n = columns[0].size();
    for (const auto& col : columns)
    {
        if (col.size() != n)
            throw std::runtime_error("write_csv: column sizes do not match");
    }

    // Open the output file for writing.
    std::ofstream out(filename);
    if (!out)
        throw std::runtime_error("write_csv: failed to open file: " + filename);

    // Write header row.
    for (std::size_t i = 0; i < header.size(); ++i)
    {
        out << header[i];
        if (i + 1 < header.size()) out << ",";
    }
    out << "\n";

    // Write each data row.
    for (std::size_t row = 0; row < n; ++row)
    {
        for (std::size_t c = 0; c < columns.size(); ++c)
        {
            out << columns[c][row];
            if (c + 1 < columns.size()) out << ",";
        }
        out << "\n";
    }
}

} // namespace common
