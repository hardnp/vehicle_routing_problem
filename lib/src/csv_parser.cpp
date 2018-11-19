#include "csv_parser.h"
#include "src/internal/utils.h"
#include "src/internal/table_parsers.h"

#include <fstream>
#include <utility>
#include <map>
#include <algorithm>
#include <stdexcept>

namespace {
/// Specifies unused variable
#define UNUSED(x) (void)x;

/// Reads file content into memory
std::pair<std::vector<std::string>,
std::map<std::string, std::pair<uint64_t, uint64_t>>>
read_file(std::ifstream& stream) {
    std::vector<std::string> content = {};
    std::vector<std::pair<std::string, uint64_t>> table_lines = {};
    std::string line = "";
    uint64_t line_number = 0;
    while (!stream.eof()) {
        std::getline(stream, line);
        if (line.empty()) continue;
        auto split_line = vrp::detail::split(line, ',');
        if (split_line[0] == "table") {
            table_lines.push_back(std::make_pair(split_line[1], line_number));
        }
        content.push_back(std::move(line));
        line_number++;
    }
    std::sort(table_lines.begin(), table_lines.end(),
        [] (const auto& a, const auto& b) { return a.second < b.second; });
    table_lines.push_back(std::make_pair("_", content.size()));
    std::map<std::string, std::pair<uint64_t, uint64_t>> table_ranges = {};
    for (size_t i = 0; i < table_lines.size() - 1; ++i) {
        table_ranges[table_lines[i].first] = std::make_pair(
            table_lines[i].second, table_lines[i + 1].second);
    }
    return std::make_pair(content, table_ranges);
}

/// Checks that all tables have corresponding range
void check_all_tables_exist(std::vector<std::string> all_tables,
    const std::map<std::string, std::pair<uint64_t, uint64_t>>& table_ranges) {
    for (const auto& name : all_tables) {
        if (table_ranges.cend() == std::find_if(
            table_ranges.cbegin(), table_ranges.cend(),
            [&name](const auto& pair) { return name == pair.first; })) {
            auto msg = std::string("table ") + name + std::string(
                " not found");
            throw std::runtime_error(msg);
        }
    }
}
}  // anonymous

namespace vrp {
CsvParser::CsvParser(const std::string& file_path) : path(file_path) {}

CsvParser::ParsedInputData CsvParser::load_input() const {
    std::ifstream file_stream(this->path);
    auto read_data = read_file(file_stream);
    auto& content = read_data.first;
    auto& table_ranges = read_data.second;
    file_stream.close();
    check_all_tables_exist({
        detail::CustomerTableParser::table_name,
        detail::VehicleTableParser::table_name,
        detail::CostTableParser::table_name,
        detail::TimeTableParser::table_name},
        table_ranges);
    CsvParser::ParsedInputData loaded = {};
    loaded.customers = detail::CustomerTableParser(content,
        table_ranges.at("customer"), 6).get();
    loaded.vehicles = detail::VehicleTableParser(content,
        table_ranges.at("vehicle"), 4).get();
    loaded.costs = detail::CostTableParser(content,
        table_ranges.at("cost"), loaded.customers.size()).get();
    loaded.times = detail::TimeTableParser(content,
        table_ranges.at("time"), loaded.customers.size()).get();
    return loaded;
}

void CsvParser::save_output(const Solution& sln) const {
    UNUSED(sln)
    return;
}
}  // vrp
