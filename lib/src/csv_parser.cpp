#include "csv_parser.h"
#include "objective.h"
#include "src/internal/subparsers.h"

#include <algorithm>
#include <cassert>
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
        if (line.length() >= 5 && std::equal(line.cbegin(), line.cbegin() + type.size(),
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
read_file(std::istream& stream) {
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
void check_all_values_exist(
    std::vector<std::string> names,
    const std::map<std::string, std::pair<uint64_t, uint64_t>>& data_ranges) {
    for (const auto& name : names) {
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

std::ostream& operator<<(std::ostream& out, const vrp::RoutePointTime& t) {
    return out << t.arrive << ';' << t.start << ';' << t.finish << ';';
}
}  // namespace

namespace vrp {
CsvParser::CsvParser(char delimiter) : m_delimiter(delimiter) {}

Problem CsvParser::read(std::istream& in) const {
    auto read_data = read_file(in);
    auto& content = read_data.first;
    auto& data_ranges = read_data.second;
    check_all_values_exist({detail::CustomerTableParser::table_name,
                            detail::VehicleTableParser::table_name,
                            detail::CostTableParser::table_name,
                            detail::TimeTableParser::table_name,
                            "max_violated_soft_tw", "max_splits"},
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
    problem.max_splits =
        std::max(problem.max_splits,
                 detail::IntValueParser(content, data_ranges.at("max_splits"),
                                        1, this->m_delimiter)
                     .get());
    problem.set_up();

    return problem;
}

// TODO: verify that this works
void CsvParser::write(std::ostream& out, const Problem& prob,
                      const Solution& sln) const {

    out << "Route;"
        << "Vehicle;"
        << "Customer;"
        << "Arrival;"
        << "Start;"
        << "Finish and leave;"
        << "Distance from previous;"
        << "Distance from depot\n";

    char del = this->m_delimiter;

    static const auto at = [](const auto& list, size_t i) {
        if (list.size() <= i) {
            throw std::out_of_range("index out of list's range");
        }
        return *std::next(list.begin(), i);
    };

    // calculate distance from customer (i - 1) to customer i
    auto prev_dist = [&prob](const auto& route, size_t i) {
        return prob.costs[at(route.second, i - 1)][at(route.second, i)];
    };

    assert(sln.routes.size() == sln.times.size());
    for (size_t i = 0; i < sln.routes.size(); ++i) {
        const auto& route = sln.routes[i];
        const auto& time = sln.times[i];

        assert(route.second.size() == time.second.size());
        // node with j == 0 is a depot
        for (size_t j = 1; j < route.second.size() - 1; ++j) {
            auto c = at(route.second, j);  // customer

            out << i << del << route.first << del << c << del;
            out << at(time.second, j);
            out << prev_dist(route, j) << del << prob.costs[c][0] << "\n";
        }
    }
}
}  // namespace vrp
