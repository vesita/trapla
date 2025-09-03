#include "ground/ground.hpp"

Ground::Ground(std::string filename) {
    Reader reader;
    try {
        if (!reader.readFromFile(filename)) {
            std::cout << "Error: Could not read file " << filename << std::endl;
            return;
        } else {
            map = reader.getData();
        }
    } catch (std::exception& e) {
        std::cout << "Error: " << e.what() << std::endl;
        return;
    }
}
