#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <random>

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

sf::RenderWindow& initialize(double size) {
  sf::VideoMode vm = sf::VideoMode::getDesktopMode();
  sf::ContextSettings settings;
  settings.antialiasingLevel = 0;
  static sf::RenderWindow window(sf::VideoMode(vm.width / size, vm.height / size), "Vehicle Routing Problem", sf::Style::Titlebar | sf::Style::Close | sf::Style::Resize, settings);
  window.setPosition(sf::Vector2i(vm.width / 2 - vm.width / (size * 2), vm.height / 2 - vm.height / (size * 2)));
  window.setKeyRepeatEnabled(1);
  window.setVerticalSyncEnabled(1);
  return window;
}

class SolutionDrawer {
  static constexpr auto ZOOM_FACTOR = 0.9;
public:
  SolutionDrawer() = delete;
  SolutionDrawer(SolutionDrawer&&) = delete;
  SolutionDrawer(SolutionDrawer const&) = delete;
  SolutionDrawer(sf::RenderWindow& window): window {window} {}

  auto solution_parse(std::string const& filename) {

    std::fstream fstream {filename, std::ios_base::in};
    std::string current_line;
    std::size_t route, vehicle, customer;
    // !@note Skipping the solution's header.
    std::getline(fstream, current_line); 

    while (std::getline(fstream, current_line)) {
      std::replace(std::begin(current_line), std::end(current_line), ';', ' '); // Replace the commas by spaces.
      std::istringstream iss (current_line);
      if (iss >> route >> vehicle >> customer) {
          route_index.emplace_back(route);
          customers_id.emplace_back(customer);
      }
    }

    // !@note In order to be drawn correctly, each route has to be represented as a vector.
    //        Here the routes are being re-constructed, in order to be processed later on.
    routes.resize(*std::max_element(std::begin(route_index), std::end(route_index)) + 1);
    for (std::size_t index = 0; index < route_index.size(); ++index) 
      routes[route_index[index]].push_back(customers_id[index]);

    for (auto& route: routes) {
      // !@note In order to be drawn correctly, we should append the depot destination. 
      route.insert(std::begin(route), 0); 
      route.insert(std::end(route), 0); 
    }

  }

  auto solomon_instance_parse(std::string const& filename) {

    std::fstream fstream {filename, std::ios_base::in};
    std::string current_line;

    std::size_t vehicle_amount, capacity;
    while (std::getline(fstream, current_line)) {
      std::istringstream iss (current_line);
      if (iss >> vehicle_amount >> capacity) break; // Skipping the vehicle/capacity line.
    }

    std::size_t id, x, y, demand;
    while (std::getline(fstream, current_line)) {
      std::istringstream iss (current_line);
      if (iss >> id >> x >> y >> demand) customers.emplace_back(CustomerData {id, x, y, demand});
    }

    for (std::size_t index = 0; index < customers.size(); ++index)
        std::tie(xmax, ymax, xmin, ymin) =
        std::tie(std::max(xmax, customers[index].x), std::max(ymax, customers[index].y),
                 std::min(xmin, customers[index].x), std::min(ymin, customers[index].y));
  }

  auto draw() {

    static std::random_device random_device;
    static unsigned seed = random_device();

    window.clear(sf::Color::Black);

    std::mt19937 PRNG {seed};
    sf::Vector2u size = window.getSize();

    auto map = [size, this](auto cx, auto cy) -> sf::Vector2<std::size_t> {
        /** @note Map the values to the range [0, 1]. */
        const auto nx = (cx - xmin) / static_cast<double>(xmax - xmin);
        const auto ny = (cy - ymin) / static_cast<double>(ymax - ymin);
        const std::size_t x = nx * size.x * ZOOM_FACTOR + (1.0 - ZOOM_FACTOR) * size.x / 2;
        const std::size_t y = ny * size.y * ZOOM_FACTOR + (1.0 - ZOOM_FACTOR) * size.y / 2;
        return {x, y};
    };

    for (auto const& route: routes) {
        std::vector<sf::Vertex> line {route.size()};
        sf::Uint8 R = PRNG() % 256, G = PRNG() % 256, B = PRNG() % 256;
        sf::Color color (R, G, B);
        std::size_t index = 0;
        for (auto const& customer: route) {
            sf::Vector2<std::size_t> mapped = map(customers[customer].x, customers[customer].y);
            sf::CircleShape circle;
            auto demand_radius = (customers[customer].demand / 10.0) + 1.0;
            circle.setRadius(demand_radius);
            circle.setPosition(static_cast<double>(mapped.x - demand_radius), static_cast<double>(mapped.y - demand_radius));
            window.draw(circle);
            line[index].position = sf::Vector2f(mapped.x, mapped.y);
            line[index].color = color;
            ++index;
        }
        window.draw(line.data(), line.size(), sf::LineStrip);
    }

    static constexpr auto SQUARE_SIZE = 12;
    sf::RectangleShape rectangle (sf::Vector2f(SQUARE_SIZE, SQUARE_SIZE));
    sf::Vector2<std::size_t> mapped = map(customers[0].x, customers[0].y);
    rectangle.setPosition(mapped.x - SQUARE_SIZE / 2, mapped.y - SQUARE_SIZE / 2);
    window.draw(rectangle);

    window.display();
  }

private:

  struct CustomerData {
      CustomerData(std::size_t id, std::size_t x, std::size_t y, std::size_t demand): id {id}, x {x}, y {y}, demand {demand} {}
      std::size_t id, x, y, demand;
  };

  std::vector<std::size_t>              route_index;
  std::vector<std::size_t>              customers_id;
  std::vector<CustomerData>               customers;
  std::vector<std::vector<std::size_t>> routes;

  sf::RenderWindow&    window;
  sf::VideoMode        video_mode;
  sf::ContextSettings  settings;

  std::size_t xmax = std::numeric_limits<std::size_t>::min(),
               xmin = std::numeric_limits<std::size_t>::max(),
               ymax = std::numeric_limits<std::size_t>::min(),
               ymin = std::numeric_limits<std::size_t>::max();
};

int main(int argc, char* argv[]) {
  if (argc <= 2) { 
    std::clog << "[vrp_drawer]: missing args\nargs: solution [.sol], instance [.txt]\n"; 
    return 0;
  }
  auto& window = initialize(1.5);
  SolutionDrawer drawer (window);
  drawer.solomon_instance_parse(argv[2]);
  drawer.solution_parse(argv[1]);
  while (window.isOpen()) { drawer.draw(); }
  return 0;
}
