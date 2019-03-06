#include "csv_parser.h"
#include "src/internal/subparsers.h"

#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace {
/// Specifies unused variable
#define UNUSED(x) (void)x;

/// Checks whether given string is a supported type specifier
bool type_specifier(std::string& line) {
    static std::vector<std::string> supported_types = {"table", "value"};
    for (const auto& type : supported_types) {
        if (std::equal(line.cbegin(), line.cbegin() + type.size(),
                       type.cbegin())) {
            // remove type specifier from string
            line = {line.begin() + type.size() + 1, line.end()};
            return true;
        }
    }
    return false;
}

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
        if (line.empty())
            continue;
        if (type_specifier(line)) {
            table_lines.push_back(std::make_pair(line, line_number));
        }
        content.push_back(std::move(line));
        line_number++;
    }
    table_lines.push_back(std::make_pair("_", content.size()));
    std::map<std::string, std::pair<uint64_t, uint64_t>> data_ranges = {};
    for (size_t i = 0; i < table_lines.size() - 1; ++i) {
        data_ranges[table_lines[i].first] =
            std::make_pair(table_lines[i].second, table_lines[i + 1].second);
    }
    return std::make_pair(content, data_ranges);
}

/// Checks that all tables have corresponding range
void check_all_tables_exist(
    std::vector<std::string> all_tables,
    const std::map<std::string, std::pair<uint64_t, uint64_t>>& data_ranges) {
    for (const auto& name : all_tables) {
        if (data_ranges.cend() == std::find_if(data_ranges.cbegin(),
                                               data_ranges.cend(),
                                               [&name](const auto& pair) {
                                                   return name == pair.first;
                                               })) {
            std::stringstream ss;
            ss << "table " << name << " not found";
            throw std::runtime_error(ss.str());
        }
    }
}
}  // namespace

namespace vrp {
CsvParser::CsvParser(const std::string& file_path, char delimiter)
    : m_delimiter(delimiter), m_csv_file(std::ifstream(file_path)) {}

CsvParser::~CsvParser() { m_csv_file.close(); }

Problem CsvParser::load_input() const {
    auto read_data = read_file(this->m_csv_file);
    auto& content = read_data.first;
    auto& data_ranges = read_data.second;
    check_all_tables_exist({detail::CustomerTableParser::table_name,
                            detail::VehicleTableParser::table_name,
                            detail::CostTableParser::table_name,
                            detail::TimeTableParser::table_name},
                           data_ranges);
    Problem problem = {};
    problem.customers =
        detail::CustomerTableParser(content, data_ranges.at("customer"), 7,
                                    this->m_delimiter)
            .get();
    problem.vehicles =
        detail::VehicleTableParser(content, data_ranges.at("vehicle"), 5,
                                   this->m_delimiter)
            .get();
    problem.costs =
        detail::CostTableParser(content, data_ranges.at("cost"),
                                problem.customers.size(), this->m_delimiter)
            .get();
    problem.times =
        detail::TimeTableParser(content, data_ranges.at("time"),
                                problem.customers.size(), this->m_delimiter)
            .get();
    problem.max_violated_soft_tw =
        detail::IntValueParser(content, data_ranges.at("max_violated_soft_tw"),
                               1, this->m_delimiter)
            .get();
    problem.set_up();
    return problem;
}

// TODO: verify that this works
void CsvParser::save_output(const std::string& out_path, const Problem& prb,
                            const Solution& sln) const {

    std::ofstream outfile(out_path + std::string("/solution.csv"));

    outfile << "Route;"
            << "Vehicle;"
            << "Customer;"
            << "Arrival;"
            << "Start;"
            << "Finish and leave;"
            << "Distance from previous;"
            << "Distance from depot\n";

    char del = ';';

    static const auto at = [](const auto& list, size_t i) {
        if (list.size() <= i) {
            throw std::out_of_range("index out of list's range");
        }
        return *std::next(list.begin(), i);
    };

    // for calculating distance from previous customer
    auto prev_dist = [&prb](const auto& route, size_t i) {
        return prb.costs[at(route.second, i)][at(route.second, i - 1)];
    };

    for (size_t i = 0; i < sln.routes.size(); ++i) {
        const auto& route = sln.routes[i];

        for (size_t j = 1; j < route.second.size() - 1;
             ++j) {  // assuming the first node is depot
            double prev_cust_dist = prev_dist(route, j);

            auto veh_id = prb.vehicles[route.first].id;
            outfile << i << del << veh_id << del << at(route.second, j) << del;

            auto find_veh_id = [&veh_id](const auto& split) {
                return split.first == veh_id;
            };
            // seek for current vehicle in split-delivery
            auto times = std::find_if(sln.times[i].begin(), sln.times[i].end(),
                                      find_veh_id);

            outfile << (*times).second;

            outfile << prev_cust_dist << del
                    << prb.costs[at(route.second, j)][0] << "\n";
        }
    }

    outfile.close();
    return;
}

void CsvParser::print_output(const Solution& sln) const {
    UNUSED(sln)
    return;
}
}  // namespace vrp
